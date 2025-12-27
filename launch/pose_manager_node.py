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
import yaml
import os
import math

class PoseManagerNode(Node):
    def __init__(self):
        super().__init__('pose_manager_node')

        # Path ke saved poses
        self.saved_poses_file = '/home/irfan/ros2_ws/src/protobot/config/saved_poses.yaml'

        # Buat fail jika belum wujud
        if not os.path.exists(self.saved_poses_file):
            with open(self.saved_poses_file, 'w') as f:
                yaml.dump({}, f)

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action Clients
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')  # Untuk smart path

        # State
        self.request_queue = []                  # Contoh: ['table1', 'table5', 'table12']
        self.is_navigating = False
        self.current_target_name = None
        self.current_goal_handle = None

        # Subscriptions
        self.create_subscription(String, '/save_pose', self.save_pose_callback, 10)
        self.create_subscription(String, '/go_to_pose', self.go_to_pose_callback, 10)
        self.create_subscription(String, '/cancel_pose', self.cancel_pose_callback, 10)
        self.create_subscription(String, '/show_queue', self.show_queue_callback, 10)

        # Timers
        self.create_timer(1.0, self.process_queue)    # Process queue setiap 1s
        self.create_timer(1.0, self.monitor_pose)     # Monitor posisi robot

        self.get_logger().info('Pose Manager STARTED → Smart Path Distance (Nav2 Planner) + Cancel Support')

    # ==================================================================
    # MONITOR & SAVE POSE
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
            self.get_logger().warn("Nama pose kosong!")
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
        if name not in self.request_queue:
            self.request_queue.append(name)
            self.get_logger().info(f"Request baru → '{name}' ditambah ke queue")
        self.show_queue_callback(None)

    def cancel_pose_callback(self, msg):
        name = msg.data.strip()
        if not name:
            self.get_logger().warn("Cancel: Nama kosong!")
            return

        if name in self.request_queue:
            self.request_queue.remove(name)
            self.get_logger().info(f"❌ Request '{name}' dibatalkan dari queue")

        if self.is_navigating and self.current_target_name == name and self.current_goal_handle:
            self.get_logger().warn(f"🔴 Membatalkan navigasi semasa ke '{name}'...")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_response_callback)

        self.show_queue_callback(None)

    def cancel_response_callback(self, future):
        self.is_navigating = False
        self.current_goal_handle = None
        self.current_target_name = None
        self.get_logger().info("Navigasi semasa dibatalkan")
        self.process_queue()

    # ==================================================================
    # SMART QUEUE: Gunakan Actual Path Distance dari Nav2
    # ==================================================================
    def process_queue(self):
        if self.is_navigating or not self.request_queue:
            return

        # Tunggu server ComputePathToPose
        if not self.path_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("ComputePathToPose server tidak ready → guna FIFO")
            self.process_queue_fifo()
            return

        # Baca saved poses
        try:
            with open(self.saved_poses_file, 'r') as f:
                saved_poses = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f"Gagal baca poses: {e} → guna FIFO")
            self.process_queue_fifo()
            return

        candidates = []

        for name in self.request_queue[:]:
            if name not in saved_poses:
                self.get_logger().warn(f"Pose '{name}' tiada dalam fail – diabaikan")
                continue

            pose_data = saved_poses[name]['position']
            orient_data = saved_poses[name]['orientation']

            # Buat goal untuk compute path
            compute_goal = ComputePathToPose.Goal()
            compute_goal.goal.header.frame_id = 'map'
            compute_goal.goal.header.stamp = self.get_clock().now().to_msg()
            compute_goal.goal.pose.position.x = pose_data['x']
            compute_goal.goal.pose.position.y = pose_data['y']
            compute_goal.goal.pose.position.z = pose_data.get('z', 0.0)
            compute_goal.goal.pose.orientation.x = orient_data.get('x', 0.0)
            compute_goal.goal.pose.orientation.y = orient_data.get('y', 0.0)
            compute_goal.goal.pose.orientation.z = orient_data.get('z', 0.0)
            compute_goal.goal.pose.orientation.w = orient_data.get('w', 1.0)

            # Hantar dan tunggu result (sync)
            send_future = self.path_client.send_goal_async(compute_goal)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=4.0)
            if not send_future.done() or not send_future.result().accepted:
                continue

            goal_handle = send_future.result()
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
            if not result_future.done():
                continue

            path = result_future.result().result.path
            if len(path.poses) < 2:
                continue  # Tiada laluan valid

            # Kira panjang path sebenar
            total_dist = 0.0
            for i in range(1, len(path.poses)):
                p1 = path.poses[i-1].pose.position
                p2 = path.poses[i].pose.position
                total_dist += math.hypot(p2.x - p1.x, p2.y - p1.y)

            candidates.append((name, total_dist))

        # Jika tiada candidates → fallback FIFO
        if not candidates:
            self.get_logger().warn("Tiada laluan valid ke mana-mana → guna FIFO")
            self.process_queue_fifo()
            return

        # Pilih yang paling pendek
        candidates.sort(key=lambda x: x[1])
        best_name, best_dist = candidates[0]

        self.get_logger().info(f"🧠 SMART PATH: Pilih '{best_name}' (laluan sebenar: {best_dist:.2f} m)")
        if len(candidates) > 1:
            others = " | ".join([f"{n} ({d:.2f}m)" for n, d in candidates[1:]])
            self.get_logger().info(f"   Alternatif: {others}")

        # Keluarkan dari queue dan navigasi
        self.request_queue.remove(best_name)
        self.current_target_name = best_name
        self.send_navigation_goal(best_name)
        self.show_queue_callback(None)

    def process_queue_fifo(self):
        """Fallback: Ikut turutan masuk (FIFO)"""
        if not self.request_queue:
            return
        next_name = self.request_queue.pop(0)
        self.get_logger().info(f"📋 FIFO: Menuju ke '{next_name}'")
        self.current_target_name = next_name
        self.send_navigation_goal(next_name)
        self.show_queue_callback(None)

    # ==================================================================
    # NAVIGATION
    # ==================================================================
    def send_navigation_goal(self, name):
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
                self.get_logger().error("Nav2 server tidak ready!")
                return

            self.is_navigating = True
            future = self.nav_client.send_goal_async(goal)
            future.add_done_callback(self.goal_response_callback)
        except Exception as e:
            self.get_logger().error(f"Gagal hantar goal: {e}")
            self.is_navigating = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal ditolak oleh Nav2!")
            self.is_navigating = False
            self.current_goal_handle = None
            self.current_target_name = None
            self.process_queue()
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info(f"🔵 Menuju ke '{self.current_target_name}'...")
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.is_navigating = False
        self.current_goal_handle = None
        self.current_target_name = None

        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info("✅ Berjaya sampai destinasi!")
        else:
            self.get_logger().warn(f"⚠️ Navigasi gagal (code: {result.error_code})")

        self.process_queue()

    # ==================================================================
    # DISPLAY QUEUE
    # ==================================================================

    def show_queue_callback(self, msg):
        """Display queue dengan ACTUAL PATH DISTANCE dari Nav2 Planner"""
        if not self.request_queue and not self.is_navigating:
            self.get_logger().info("Queue KOSONG – tiada permintaan.")
            return

        # Tunggu server ComputePathToPose
        if not self.path_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("ComputePathToPose server tidak ready – papar queue tanpa jarak actual")
            self._show_queue_fallback()
            return

        # Baca saved poses
        try:
            with open(self.saved_poses_file, 'r') as f:
                saved_poses = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f"Gagal baca poses: {e}")
            self._show_queue_fallback()
            return

        self.get_logger().info("=== QUEUE SEMASA (jarak laluan sebenar) ===")

        candidates = []

        for name in self.request_queue:
            if name not in saved_poses:
                candidates.append((name, "UNKNOWN (pose tiada)"))
                continue

            pose_data = saved_poses[name]
            pos = pose_data['position']
            ori = pose_data['orientation']

            compute_goal = ComputePathToPose.Goal()
            compute_goal.goal.header.frame_id = 'map'
            compute_goal.goal.header.stamp = self.get_clock().now().to_msg()
            compute_goal.goal.pose.position.x = pos['x']
            compute_goal.goal.pose.position.y = pos['y']
            compute_goal.goal.pose.position.z = pos.get('z', 0.0)
            compute_goal.goal.pose.orientation.x = ori.get('x', 0.0)
            compute_goal.goal.pose.orientation.y = ori.get('y', 0.0)
            compute_goal.goal.pose.orientation.z = ori.get('z', 0.0)
            compute_goal.goal.pose.orientation.w = ori.get('w', 1.0)

            # Hantar request compute path
            send_future = self.path_client.send_goal_async(compute_goal)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=3.0)
            if not send_future.done() or not send_future.result().accepted:
                candidates.append((name, "GAGAL PLAN"))
                continue

            goal_handle = send_future.result()
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=4.0)
            if not result_future.done():
                candidates.append((name, "TIMEOUT"))
                continue

            path = result_future.result().result.path
            if len(path.poses) < 2:
                candidates.append((name, "TIADA LALUAN"))
                continue

            # Kira actual path distance
            total_dist = 0.0
            for i in range(1, len(path.poses)):
                p1 = path.poses[i-1].pose.position
                p2 = path.poses[i].pose.position
                total_dist += math.hypot(p2.x - p1.x, p2.y - p1.y)

            candidates.append((name, f"{total_dist:.2f} m"))

        # Papar dengan nombor dan status
        for i, (name, dist_str) in enumerate(candidates, 1):
            status = ""
            if self.is_navigating and name == self.current_target_name:
                status = " ← SEDANG MENUJU"
            elif not self.is_navigating and i == 1:
                status = " ← AKAN PERGI SETERUSNYA"

            self.get_logger().info(f"{i}. {name} → {dist_str}{status}")

        # Jika sedang navigasi ke pose yang tak ada dalam queue lagi
        if self.is_navigating and self.current_target_name not in self.request_queue:
            self.get_logger().info(f"SEDANG MENUJU ke '{self.current_target_name}' (sedang dalam perjalanan)")

    def _show_queue_fallback(self):
        """Fallback display tanpa actual path (hanya nama + status)"""
        self.get_logger().info("=== QUEUE SEMASA (tanpa jarak – planner tidak ready) ===")
        for i, name in enumerate(self.request_queue, 1):
            status = ""
            if self.is_navigating and name == self.current_target_name:
                status = " ← SEDANG MENUJU"
            elif not self.is_navigating and i == 1:
                status = " ← AKAN PERGI SETERUSNYA"
            self.get_logger().info(f"{i}. {name}{status}")


def main(args=None):
    rclpy.init(args=args)
    node = PoseManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()