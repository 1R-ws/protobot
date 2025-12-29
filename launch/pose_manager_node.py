#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import math


class PoseManagerNode(Node):

    def __init__(self):
        super().__init__('pose_manager_node')

        # --------------------------------------------------
        # File path
        # --------------------------------------------------
        pkg_path = get_package_share_directory('protobot')
        self.saved_poses_file = os.path.join(pkg_path, 'config', 'saved_poses.yaml')
        os.makedirs(os.path.dirname(self.saved_poses_file), exist_ok=True)
        if not os.path.exists(self.saved_poses_file):
            with open(self.saved_poses_file, 'w') as f:
                yaml.dump({}, f)

        # --------------------------------------------------
        # TF
        # --------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --------------------------------------------------
        # Action clients
        # --------------------------------------------------
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        # --------------------------------------------------
        # Publishers
        # --------------------------------------------------
        self.queue_pub = self.create_publisher(String, '/queue_status', 10)
        self.pose_pub = self.create_publisher(String, '/robot_pose', 10)

        # --------------------------------------------------
        # State
        # --------------------------------------------------
        self.request_queue = []
        self.distance_cache = {}
        self.pending_path = {}
        self.is_navigating = False
        self.current_target = None
        self.current_goal_handle = None
        self.queue_wait_start = None

        # --------------------------------------------------
        # Subscribers
        # --------------------------------------------------
        self.create_subscription(String, '/save_pose', self.save_pose_cb, 10)
        self.create_subscription(String, '/go_to_pose', self.go_to_pose_cb, 10)
        self.create_subscription(String, '/cancel_pose', self.cancel_pose_cb, 10)
        self.create_subscription(String, '/show_queue', self.show_queue_cb, 10)

        # --------------------------------------------------
        # Timers
        # --------------------------------------------------
        self.create_timer(2.0, self.process_queue)
        self.create_timer(1.0, self.publish_robot_pose)

        self.get_logger().info("Pose Manager READY (FULL FIXED VERSION)")

    # ==================================================
    # ROBOT POSE MONITOR
    # ==================================================
    def publish_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
            p = tf.transform.translation
            o = tf.transform.rotation
            msg = f"x:{p.x:.3f}, y:{p.y:.3f} | qx:{o.x:.3f}, qy:{o.y:.3f}, qz:{o.z:.3f}, qw:{o.w:.3f}"
            self.pose_pub.publish(String(data=msg))
        except TransformException:
            pass

    # ==================================================
    # SAVE POSE
    # ==================================================
    def save_pose_cb(self, msg):
        name = msg.data.strip()
        if not name:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            p = tf.transform.translation
            o = tf.transform.rotation
            with open(self.saved_poses_file, 'r') as f:
                data = yaml.safe_load(f) or {}
            data[name] = {
                'position': {'x': p.x, 'y': p.y, 'z': p.z},
                'orientation': {'x': o.x, 'y': o.y, 'z': o.z, 'w': o.w}
            }
            with open(self.saved_poses_file, 'w') as f:
                yaml.dump(data, f)
            self.get_logger().info(f"Pose '{name}' disimpan")
        except TransformException:
            self.get_logger().error("TF lookup gagal")

    # ==================================================
    # QUEUE CONTROL
    # ==================================================
    def go_to_pose_cb(self, msg):
        name = msg.data.strip()
        if name and name not in self.request_queue:
            self.request_queue.append(name)
            self.distance_cache.pop(name, None)
            self.queue_wait_start = None
            self.publish_queue_status()

    def cancel_pose_cb(self, msg):
        name = msg.data.strip()
        if name in self.request_queue:
            self.request_queue.remove(name)
        if self.is_navigating and name == self.current_target and self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
        self.publish_queue_status()

    # ==================================================
    # SMART QUEUE PROCESSING
    # ==================================================
    def process_queue(self):
        if self.is_navigating or not self.request_queue:
            return
        if self.queue_wait_start is None:
            self.queue_wait_start = self.get_clock().now()

        # Request missing distances
        for name in self.request_queue:
            if name not in self.distance_cache and name not in self.pending_path:
                self.request_path_distance(name)

        # Check if all distances ready
        if not self.all_distances_ready():
            elapsed = (self.get_clock().now() - self.queue_wait_start).nanoseconds / 1e9
            if elapsed < 5.0:
                return
            else:
                self.get_logger().warn("Planner timeout → FIFO fallback")
                best_name = self.request_queue.pop(0)
                self.send_navigation_goal(best_name)
                return

        # Safe selection: min actual path distance
        best_name = min(self.request_queue, key=lambda n: self.distance_cache[n])
        self.request_queue.remove(best_name)
        self.send_navigation_goal(best_name)

    def all_distances_ready(self):
        return all(name in self.distance_cache for name in self.request_queue)

    # ==================================================
    # COMPUTE PATH DISTANCE (ASYNC)
    # ==================================================
    def request_path_distance(self, name):
        try:
            with open(self.saved_poses_file) as f:
                poses = yaml.safe_load(f) or {}
            if name not in poses:
                return
            goal = ComputePathToPose.Goal()
            goal.goal.header.frame_id = 'map'
            goal.goal.pose.position.x = poses[name]['position']['x']
            goal.goal.pose.position.y = poses[name]['position']['y']
            goal.goal.pose.orientation.x = poses[name]['orientation'].get('x', 0.0)
            goal.goal.pose.orientation.y = poses[name]['orientation'].get('y', 0.0)
            goal.goal.pose.orientation.z = poses[name]['orientation'].get('z', 0.0)
            goal.goal.pose.orientation.w = poses[name]['orientation'].get('w', 1.0)

            self.pending_path[name] = True
            future = self.path_client.send_goal_async(goal)
            future.add_done_callback(lambda f, n=name: self.path_response_cb(f, n))
        except Exception:
            pass

    def path_response_cb(self, future, name):
        self.pending_path.pop(name, None)
        if not future.result() or not future.result().accepted:
            return
        gh = future.result()
        gh.get_result_async().add_done_callback(lambda f, n=name: self.path_result_cb(f, n))

    def path_result_cb(self, future, name):
        try:
            path = future.result().result.path
            dist = 0.0
            for i in range(1, len(path.poses)):
                p1 = path.poses[i-1].pose.position
                p2 = path.poses[i].pose.position
                dist += math.hypot(p2.x - p1.x, p2.y - p1.y)
            self.distance_cache[name] = dist
            self.publish_queue_status()
        except Exception:
            pass

    # ==================================================
    # NAVIGATION
    # ==================================================
    def send_navigation_goal(self, name):
        try:
            with open(self.saved_poses_file) as f:
                poses = yaml.safe_load(f) or {}
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = poses[name]['position']['x']
            goal.pose.pose.position.y = poses[name]['position']['y']
            goal.pose.pose.position.z = poses[name]['position'].get('z', 0.0)

            # ✅ Gunakan full quaternion
            goal.pose.pose.orientation.x = poses[name]['orientation'].get('x', 0.0)
            goal.pose.pose.orientation.y = poses[name]['orientation'].get('y', 0.0)
            goal.pose.pose.orientation.z = poses[name]['orientation'].get('z', 0.0)
            goal.pose.pose.orientation.w = poses[name]['orientation'].get('w', 1.0)

            self.is_navigating = True
            self.current_target = name
            self.queue_wait_start = None

            future = self.nav_client.send_goal_async(goal)
            future.add_done_callback(self.nav_goal_cb)

            self.publish_queue_status()
        except Exception as e:
            self.get_logger().error(f"Gagal hantar goal: {e}")
            self.is_navigating = False

    def nav_goal_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.is_navigating = False
            return
        self.current_goal_handle = gh
        gh.get_result_async().add_done_callback(self.nav_result_cb)

    def nav_result_cb(self, future):
        self.is_navigating = False
        self.current_goal_handle = None
        self.current_target = None
        self.publish_queue_status()

    # ==================================================
    # QUEUE STATUS PUBLISHER
    # ==================================================
    def publish_queue_status(self):
        lines = ["=== QUEUE STATUS ==="]
        if self.is_navigating and self.current_target:
            lines.append(f"ACTIVE → {self.current_target}")
        for i, name in enumerate(self.request_queue, 1):
            dist = self.distance_cache.get(name, "calculating...")
            lines.append(f"{i}. {name} → {dist}")
        self.queue_pub.publish(String(data="\n".join(lines)))

    def show_queue_cb(self, msg):
        self.publish_queue_status()


def main(args=None):
    rclpy.init(args=args)
    node = PoseManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()