#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class JoystickPose(Node):
    def __init__(self):
        super().__init__('joystick_pose')
        self.pub_save = self.create_publisher(String, '/save_pose', 10)
        self.pub_go = self.create_publisher(String, '/go_to_pose', 10)
        self.create_subscription(Joy, '/joy', self.joy_cb, 10)

        # Butang: X, O, □, △
        self.btn_save_rumah = 0   # X
        self.btn_go_rumah   = 1   # O
        self.btn_save_dapur = 2   # □
        self.btn_go_dapur   = 3   # △

        self.get_logger().info("JOYSTICK POSE: X=Simpan 'rumah', O=Pergi 'rumah', □=Simpan 'dapur', △=Pergi 'dapur'")

    def joy_cb(self, msg):
        b = msg.buttons
        if b[self.btn_save_rumah] == 1:
            self.pub_save.publish(String(data='rumah'))
            self.get_logger().info("SIMPAN: rumah")
        if b[self.btn_go_rumah] == 1:
            self.pub_go.publish(String(data='rumah'))
            self.get_logger().info("PERGI: rumah")
        if b[self.btn_save_dapur] == 1:
            self.pub_save.publish(String(data='dapur'))
            self.get_logger().info("SIMPAN: dapur")
        if b[self.btn_go_dapur] == 1:
            self.pub_go.publish(String(data='dapur'))
            self.get_logger().info("PERGI: dapur")


def main():
    rclpy.init()
    node = JoystickPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()