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
import math
from ament_index_python.packages import get_package_share_directory


class PoseManagerNode(Node):
    def __init__(self):
        super().__init__('pose_manager_node')

        # use_sim_time dari launch file
        self.use_sim_time = self.get_parameter_or('use_sim_time', False).value

        # Path ke saved poses
        self.saved_poses_file = '/home/irfan/ros2_ws/src/protobot/config/saved_poses.yaml'

        # Buat fail jika tak wujud
        if not os.path.exists(self.saved_poses_file):
            with open(self.saved_poses_file, 'w') as f:
                yaml.dump({}, f)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Queue untuk smart navigation (nearest first)
        self.request_queue = []          # contoh: ["table1", "lecture_table", "table2"]
        self.is_navigating = False

        # Subscriptions
        self.create_subscription(String, '/save_pose', self.save_pose_callback, 10)
        self.create_subscription(String, '/go_to_pose', self.go_to_pose_callback, 10)
        self.create_subscription(String, '/show_queue', self.show_queue_callback, 10)  # Manual display

        # Timers
        self.create_timer(0.5, self.process_queue)   # Check queue setiap 0.5s
        self.create_timer(1.0, self.monitor_pose)   # Monitor posisi robot

        self.get_logger().info(f'Pose Manager STARTED → Smart queue + auto display aktif | {self.saved_poses_file}')

    # ==================================================================
    # MONITOR & SAVE POSE (original, unchanged)
    # ==================================================================
    def monitor_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=1.0))
            p = trans.transform.translation
            o = trans.transform.rotation
            self.get_logger().info(f"Posisi semasa → x: {p.x:.3f}, y: {p.y:.3f} | qz: {o.z:.3f}, qw: {o.w:.3f}")
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

            self.get_logger().info(f"Pose '{name}' berjaya disimpan!")
        except Exception as e:
            self.get_logger().error(f"Gagal simpan pose: {e}")

    # ==================================================================
    # QUEUE MANAGEMENT
    # ==================================================================
    def go_to_pose_callback(self, msg):
        name = msg.data.strip()
        if not name:
            return

        # Tambah ke queue jika belum ada (elak duplicate)
        if name not in self.request_queue:
            self.request_queue.append(name)
            self.get_logger().info(f"Request baru → '{name}' ditambah ke queue")

        # Susun semula berdasarkan jarak terkini
        self.sort_queue_by_distance()

        # Auto display queue
        self.show_queue_callback(None)

    def sort_queue_by_distance(self):
        if len(self.request_queue) < 2:
            return

        # Retry TF lookup (maksimum 3 kali)
        robot_x = robot_y = None
        for attempt in range(3):
            try:
                trans = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=2.0))
                robot_x = trans.transform.translation.x
                robot_y = trans.transform.translation.y
                break
            except TransformException:
                if attempt < 2:
                    rclpy.spin_once(self, timeout_sec=0.5)
                else:
                    self.get_logger().warn("TF lookup gagal selepas 3 percubaan – queue tidak disusun")
                    return

        try:
            with open(self.saved_poses_file, 'r') as f:
                poses = yaml.safe_load(f) or {}

            def distance(table_name):
                if table_name not in poses:
                    return float('inf')  # Jika pose tak wujud, letak akhir
                p = poses[table_name]['position']
                dist = math.hypot(p['x'] - robot_x, p['y'] - robot_y)
                return dist

            old_queue = self.request_queue.copy()
            self.request_queue.sort(key=distance)

            if old_queue != self.request_queue:
                self.get_logger().info("Queue disusun semula (terdekat dulu)")

        except Exception as e:
            self.get_logger().warn(f"Gagal susun queue: {e}")

    def process_queue(self):
        if self.is_navigating or not self.request_queue:
            return

        # Re-sort sekali lagi sebelum pop (pastikan optimal walaupun robot dah gerak)
        self.sort_queue_by_distance()

        next_table = self.request_queue.pop(0)
        self.get_logger().info(f"🔵 Menuju ke pose terdekat → '{next_table}'")
        self.send_navigation_goal(next_table)

        # Auto display queue selepas pop
        self.show_queue_callback(None)

    # ==================================================================
    # NAVIGATION
    # ==================================================================
    def send_navigation_goal(self, name):
        try:
            with open(self.saved_poses_file, 'r') as f:
                data = yaml.safe_load(f) or {}

            if name not in data:
                self.get_logger().error(f"Pose '{name}' tiada dalam fail!")
                self.is_navigating = False
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
                self.get_logger().error("Nav2 action server tidak tersedia!")
                self.is_navigating = False
                return

            self.is_navigating = True
            send_goal_future = self.nav_client.send_goal_async(goal)
            send_goal_future.add_done_callback(self.goal_response_callback)

        except Exception as e:
            self.get_logger().error(f"Gagal hantar goal: {e}")
            self.is_navigating = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal ditolak oleh Nav2!")
            self.is_navigating = False
            self.process_queue()  # Cuba next dalam queue
            return

        self.get_logger().info("Goal diterima! Sedang menavigasi...")
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.is_navigating = False
        result = future.result().result

        if result.error_code == 0:
            self.get_logger().info("✅ Berjaya sampai ke destinasi!")
        else:
            self.get_logger().warn(f"⚠️ Sampai tapi ada error code: {result.error_code}")

        # Teruskan ke pose seterusnya dalam queue
        self.process_queue()

    # ==================================================================
    # DISPLAY QUEUE
    # ==================================================================
    def show_queue_callback(self, msg):
        """Display queue semasa dengan jarak (manual atau auto call)"""
        if not self.request_queue:
            self.get_logger().info("Queue KOSONG – tiada pose menunggu.")
            return

        try:
            # Dapatkan posisi robot
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=1.0))
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y

            with open(self.saved_poses_file, 'r') as f:
                poses = yaml.safe_load(f) or {}

            self.get_logger().info("=== QUEUE SEMASA (terdekat dulu) ===")
            for i, name in enumerate(self.request_queue, 1):
                status = " ← SEDANG MENUJU" if i == 1 and self.is_navigating else ""
                if name not in poses:
                    dist_str = "UNKNOWN"
                else:
                    p = poses[name]['position']
                    dist = math.hypot(p['x'] - robot_x, p['y'] - robot_y)
                    dist_str = f"{dist:.2f} m"
                self.get_logger().info(f"{i}. {name} → {dist_str}{status}")

            if not self.is_navigating:
                self.get_logger().info(f"→ Pose seterusnya: {self.request_queue[0]}")

        except TransformException:
            self.get_logger().warn("TF tidak ready – hanya papar nama queue:")
            self.get_logger().info("Queue: " + " → ".join(self.request_queue))
        except Exception as e:
            self.get_logger().error(f"Gagal display queue: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PoseManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()