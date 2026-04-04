#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from std_msgs.msg import String, Float32
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
        # Paper low -> Go HOME first (Nav2)
        # --------------------------------------------------
        self.declare_parameter('paper_threshold', 0.20)   # unit ikut /paper_weight (kg contoh)
        self.declare_parameter('trigger_count', 5)       # berapa kali berturut2 < threshold
        self.declare_parameter('home_pose_name', 'home') # pose "home" mesti wujud dalam saved_poses.yaml

        self.paper_threshold = float(self.get_parameter('paper_threshold').value)
        self.trigger_count = int(self.get_parameter('trigger_count').value)
        self.home_pose_name = str(self.get_parameter('home_pose_name').value)

        # --------------------------------------------------
        # Idle -> Go HOME bila tiada kerja
        # --------------------------------------------------
        self.declare_parameter('idle_go_home', True)
        self.declare_parameter('idle_timeout_sec', 30.0)  # queue kosong > 30s => balik home

        self.idle_go_home = bool(self.get_parameter('idle_go_home').value)
        self.idle_timeout_sec = float(self.get_parameter('idle_timeout_sec').value)

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

        # Paper low state
        self.paper_low_count = 0
        self.paper_low_mode = False

        # Activity tracking for idle-home
        self.last_activity_time = self.get_clock().now()
        self.idle_home_sent = False

        # --------------------------------------------------
        # Subscribers
        # --------------------------------------------------
        self.create_subscription(String, '/save_pose', self.save_pose_cb, 10)
        self.create_subscription(String, '/go_to_pose', self.go_to_pose_cb, 10)
        self.create_subscription(String, '/cancel_pose', self.cancel_pose_cb, 10)
        self.create_subscription(String, '/show_queue', self.show_queue_cb, 10)
        self.create_subscription(String, '/paper_request', self.paper_request_cb, 10)
        self.create_subscription(Float32, '/paper_weight', self.paper_weight_cb, 10)

        # --------------------------------------------------
        # Timers
        # --------------------------------------------------
        self.create_timer(2.0, self.process_queue)
        self.create_timer(1.0, self.publish_robot_pose)
        self.create_timer(1.0, self.check_idle_go_home)  # ✅ idle checker

        self.get_logger().info("Pose Manager READY (Paper low + Idle -> HOME via Nav2)")

    # ==================================================
    # ACTIVITY TRACKING (IDLE TIMER RESET)
    # ==================================================
    def touch_activity(self):
        """Reset idle timer bila ada aktiviti / kerja."""
        self.last_activity_time = self.get_clock().now()
        self.idle_home_sent = False

    # ==================================================
    # IDLE CHECKER -> queue kosong lama => HOME
    # ==================================================
    def check_idle_go_home(self):
        if not self.idle_go_home:
            return

        # kalau sedang navigate, jangan kacau
        if self.is_navigating:
            return

        # kalau paper_low_mode sedang paksa HOME, jangan kacau
        if self.paper_low_mode:
            return

        # kalau masih ada kerja, jangan home
        if self.request_queue:
            return

        # kalau dah pernah queue-kan HOME utk idle, jangan ulang
        if self.idle_home_sent:
            return

        idle_sec = (self.get_clock().now() - self.last_activity_time).nanoseconds / 1e9
        if idle_sec < self.idle_timeout_sec:
            return

        # pastikan HOME pose wujud
        try:
            with open(self.saved_poses_file) as f:
                poses = yaml.safe_load(f) or {}
            if self.home_pose_name not in poses:
                self.get_logger().error(
                    f"[IDLE HOME] Home pose '{self.home_pose_name}' tak wujud. Save dulu guna /save_pose '{self.home_pose_name}'."
                )
                return
        except Exception as e:
            self.get_logger().error(f"[IDLE HOME] gagal baca saved_poses.yaml: {e}")
            return

        # Queue-kan HOME dan biar process_queue hantar goal
        if self.home_pose_name in self.request_queue:
            self.request_queue.remove(self.home_pose_name)
        self.request_queue.insert(0, self.home_pose_name)

        self.distance_cache.pop(self.home_pose_name, None)
        self.pending_path.pop(self.home_pose_name, None)

        self.idle_home_sent = True
        self.get_logger().info(f"[IDLE HOME] Queue kosong {idle_sec:.1f}s → balik HOME")
        self.publish_queue_status()

    # ==================================================
    # PAPER WEIGHT CALLBACK (trigger go home)
    # ==================================================
    def paper_weight_cb(self, msg: Float32):
        w = float(msg.data)

        if w < self.paper_threshold:
            self.paper_low_count += 1
        else:
            self.paper_low_count = 0

        if (not self.paper_low_mode) and (self.paper_low_count >= self.trigger_count):
            self.paper_low_mode = True
            self.get_logger().warning(
                f"Paper LOW! w={w:.3f} < {self.paper_threshold}. Going HOME first."
            )
            self.go_home_priority()

    def go_home_priority(self):
        # Home pose must exist
        try:
            with open(self.saved_poses_file) as f:
                poses = yaml.safe_load(f) or {}
            if self.home_pose_name not in poses:
                self.get_logger().error(
                    f"Home pose '{self.home_pose_name}' not found. Save it first using /save_pose with '{self.home_pose_name}'."
                )
                self.paper_low_mode = False
                self.paper_low_count = 0
                return
        except Exception as e:
            self.get_logger().error(f"Failed to read saved_poses.yaml: {e}")
            self.paper_low_mode = False
            self.paper_low_count = 0
            return

        # Cancel current Nav2 goal if any
        try:
            if self.current_goal_handle:
                self.current_goal_handle.cancel_goal_async()
        except Exception:
            pass

        # Put HOME at the FRONT (queue lama kekal)
        if self.home_pose_name in self.request_queue:
            self.request_queue.remove(self.home_pose_name)
        self.request_queue.insert(0, self.home_pose_name)

        # Reset caches for HOME so planner compute again
        self.distance_cache.pop(self.home_pose_name, None)
        self.pending_path.pop(self.home_pose_name, None)

        # Reset navigation state (biar process_queue hantar goal home)
        self.is_navigating = False
        self.current_target = None
        self.current_goal_handle = None
        self.queue_wait_start = None

        self.touch_activity()
        self.publish_queue_status()

    # ==================================================
    # PAPER REQUEST CALLBACK (from ESP32)
    # ==================================================
    def paper_request_cb(self, msg):
        """
        Terima arahan dari /paper_request.
        Support 2 format:
        1) "request table1" / "cancel table2"
        2) "q 1" / "f 2"   (q=request, f=cancel)
        """
        try:
            s = msg.data.strip()
            if not s:
                return

            parts = s.split()
            if len(parts) != 2:
                self.get_logger().warning(f"Format msg salah: '{s}'")
                return

            action, target = parts[0].lower(), parts[1].lower()

            # Convert q/f -> request/cancel
            if action == "q":
                action = "request"
            elif action == "f":
                action = "cancel"

            # Normalize target -> tableX
            if target.isdigit():
                table_name = f"table{target}"
            else:
                table_name = target

            if action == "request":
                if table_name not in self.request_queue:
                    self.request_queue.append(table_name)
                    self.distance_cache.pop(table_name, None)
                    self.queue_wait_start = None
                    self.touch_activity()
                    self.publish_queue_status()
                    self.get_logger().info(f"📥 Queue updated: REQUEST {table_name}")

            elif action == "cancel":
                if table_name in self.request_queue:
                    self.request_queue.remove(table_name)

                if self.is_navigating and table_name == self.current_target and self.current_goal_handle:
                    self.current_goal_handle.cancel_goal_async()

                self.publish_queue_status()
                self.get_logger().info(f"📤 Queue updated: CANCEL {table_name}")

            else:
                self.get_logger().warning(f"Action tidak dikenali: {action} (msg='{s}')")

        except Exception as e:
            self.get_logger().error(f"paper_request_cb error: {e}")

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
            self.touch_activity()
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
            if name not in poses:
                self.get_logger().error(f"Pose '{name}' not found in saved_poses.yaml")
                return

            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = poses[name]['position']['x']
            goal.pose.pose.position.y = poses[name]['position']['y']
            goal.pose.pose.position.z = poses[name]['position'].get('z', 0.0)

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
        if not gh or not gh.accepted:
            self.is_navigating = False
            return
        self.current_goal_handle = gh
        gh.get_result_async().add_done_callback(self.nav_result_cb)

    def nav_result_cb(self, future):
        finished = self.current_target

        self.is_navigating = False
        self.current_goal_handle = None
        self.current_target = None

        # Reset idle timer bila navigation tamat
        self.touch_activity()

        # If we just arrived HOME (paper low), resume normal queue mode
        if self.paper_low_mode and finished == self.home_pose_name:
            self.paper_low_mode = False
            self.paper_low_count = 0
            self.get_logger().info("Arrived HOME (paper low). Resume normal queue.")

        self.publish_queue_status()

    # ==================================================
    # QUEUE STATUS PUBLISHER
    # ==================================================
    def publish_queue_status(self):
        lines = ["=== QUEUE STATUS ==="]

        if self.is_navigating and self.current_target:
            if self.paper_low_mode and self.current_target == self.home_pose_name:
                lines.append("ACTIVE → HOME (paper low)")
            else:
                lines.append(f"ACTIVE → {self.current_target}")

        if not self.request_queue:
            lines.append("(queue kosong)")

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
