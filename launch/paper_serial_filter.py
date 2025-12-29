#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class PaperSerialFilter(Node):

    def __init__(self):
        super().__init__('paper_serial_filter')

        # ===============================
        # SERIAL CONFIG
        # ===============================
        self.SERIAL_PORT = '/dev/ttyUSB0'
        self.BAUD_RATE = 115200

        # ===============================
        # LOAD saved_poses.yaml
        # ===============================
        pkg_path = get_package_share_directory('protobot')
        saved_poses_file = os.path.join(pkg_path, 'config', 'saved_poses.yaml')

        self.table_mapping = {}   # contoh: {'1': 'table1', '2': 'table2'}

        if not os.path.exists(saved_poses_file):
            self.get_logger().error(f"❌ saved_poses.yaml tak wujud: {saved_poses_file}")
            rclpy.shutdown()
            return

        try:
            with open(saved_poses_file, 'r') as f:
                data = yaml.safe_load(f)

            if data is None:
                self.get_logger().warning("⚠️ saved_poses.yaml kosong")
                data = {}

            for pose_name in data.keys():
                if pose_name.startswith('table') and pose_name[5:].isdigit():
                    num = pose_name[5:]
                    self.table_mapping[num] = pose_name
                    self.get_logger().info(f"📍 Mapping: {num} → {pose_name}")

            self.get_logger().info(
                f"✅ Load {len(self.table_mapping)} table dari saved_poses.yaml"
            )

        except Exception as e:
            self.get_logger().error(f"❌ Gagal baca saved_poses.yaml: {e}")
            rclpy.shutdown()
            return

        # ===============================
        # OPEN SERIAL
        # ===============================
        try:
            self.ser = serial.Serial(
                port=self.SERIAL_PORT,
                baudrate=self.BAUD_RATE,
                timeout=0.1
            )
            self.get_logger().info(
                f"🔌 Serial dibuka: {self.SERIAL_PORT} @ {self.BAUD_RATE}"
            )
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Tak dapat buka serial: {e}")
            rclpy.shutdown()
            return

        # ===============================
        # ROS PUBLISHER
        # ===============================
        self.publisher = self.create_publisher(
            String,
            '/paper_request',
            10
        )

        # ===============================
        # SERIAL THREAD
        # ===============================
        self.serial_thread = threading.Thread(
            target=self.read_loop,
            daemon=True
        )
        self.serial_thread.start()

    # =================================================
    # SERIAL READ LOOP (TERIMA: "q 2" / "f 2")
    # =================================================
    def read_loop(self):
        self.get_logger().info("📡 Serial read loop bermula")

        try:
            while rclpy.ok():
                if self.ser.in_waiting > 0:
                    raw = self.ser.readline().decode('utf-8', errors='ignore')

                    # buang newline & spaces
                    line = raw.strip()

                    if not line:
                        continue

                    self.get_logger().info(f"📥 SERIAL: '{line}'")

                    parts = line.split()

                    # EXPECTED: q 2  |  f 2
                    if len(parts) != 2:
                        self.get_logger().warning(f"⚠️ Format tak dikenali: {line}")
                        continue

                    cmd, table_num = parts

                    if cmd not in ['q', 'f']:
                        self.get_logger().warning(f"⚠️ Command tak sah: {cmd}")
                        continue

                    if table_num not in self.table_mapping:
                        self.get_logger().warning(
                            f"⚠️ Meja {table_num} tiada dalam saved_poses.yaml"
                        )
                        continue

                    table_name = self.table_mapping[table_num]
                    action = 'request' if cmd == 'q' else 'cancel'

                    msg = String()
                    msg.data = f"{action} {table_name}"
                    self.publisher.publish(msg)

                    self.get_logger().info(
                        f"📄 {action.upper():7} → {table_name}"
                    )

        except Exception as e:
            self.get_logger().error(f"❌ Ralat serial loop: {e}")

        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                self.get_logger().info("🔌 Serial ditutup")


def main(args=None):
    rclpy.init(args=args)
    node = PaperSerialFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
