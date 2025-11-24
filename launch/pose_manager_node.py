#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class PoseManagerNode(Node):
    def __init__(self):
        super().__init__('pose_manager_node')

        # JANGAN declare use_sim_time — dah ada dari launch
        # self.declare_parameter('use_sim_time', False)
        self.use_sim_time = self.get_parameter_or('use_sim_time', False).value

        # Dapatkan path config
        pkg_share = get_package_share_directory('protobot')
        self.saved_poses_file = os.path.join(pkg_share, 'config', 'saved_poses.yaml')

        # Pastikan fail wujud
        if not os.path.exists(self.saved_poses_file):
            with open(self.saved_poses_file, 'w') as f:
                yaml.dump({}, f)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Topics
        self.create_subscription(String, '/save_pose', self.save_pose_callback, 10)
        self.create_subscription(String, '/go_to_pose', self.go_to_pose_callback, 10)

        # Monitor
        self.create_timer(1.0, self.monitor_pose)

        self.get_logger().info(f'Pose Manager STARTED | File: {self.saved_poses_file}')

    def monitor_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=1.0))
            p = trans.transform.translation
            o = trans.transform.rotation
            self.get_logger().info(f"x: {p.x:.3f}, y: {p.y:.3f} | qz: {o.z:.3f}, qw: {o.w:.3f}")
        except TransformException:
            pass

    def save_pose_callback(self, msg):
        name = msg.data.strip()
        if not name:
            self.get_logger().warn("Nama kosong!")
            return

        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            p, o = trans.transform.translation, trans.transform.rotation

            with open(self.saved_poses_file, 'r') as f:
                data = yaml.safe_load(f) or {}

            data[name] = {
                'position': {'x': p.x, 'y': p.y, 'z': p.z},
                'orientation': {'x': o.x, 'y': o.y, 'z': o.z, 'w': o.w}
            }

            with open(self.saved_poses_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)

            self.get_logger().info(f"Pose '{name}' disimpan!")

        except Exception as e:
            self.get_logger().error(f"Gagal simpan: {e}")

    def go_to_pose_callback(self, msg):
        name = msg.data.strip()
        if not name:
            return

        try:
            with open(self.saved_poses_file, 'r') as f:
                data = yaml.safe_load(f) or {}

            if name not in data:
                self.get_logger().error(f"Pose '{name}' tiada!")
                return

            pose = data[name]
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = pose['position']['x']
            goal.pose.pose.position.y = pose['position']['y']
            goal.pose.pose.position.z = pose['position']['z']
            goal.pose.pose.orientation.x = pose['orientation']['x']
            goal.pose.pose.orientation.y = pose['orientation']['y']
            goal.pose.pose.orientation.z = pose['orientation']['z']
            goal.pose.pose.orientation.w = pose['orientation']['w']

            if not self.nav_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Nav2 server tiada!")
                return

            self.nav_client.send_goal_async(goal).add_done_callback(self.goal_cb)
            self.get_logger().info(f"Pergi ke '{name}'")

        except Exception as e:
            self.get_logger().error(f"Gagal hantar: {e}")

    def goal_cb(self, future):
        if future.result().accepted:
            self.get_logger().info("Goal diterima!")
        else:
            self.get_logger().error("Goal ditolak!")


def main():
    rclpy.init()
    node = PoseManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()