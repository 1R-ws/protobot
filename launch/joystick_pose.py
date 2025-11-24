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
        self.btn_save_table1 = 0   # X
        self.btn_go_table1   = 1   # O
        self.btn_save_table2 = 2   # □
        self.btn_go_table2   = 3   # △

        self.get_logger().info("JOYSTICK POSE: X=Simpan 'table1', O=Pergi 'table1', □=Simpan 'table2', △=Pergi 'table2'")

    def joy_cb(self, msg):
        b = msg.buttons
        if b[self.btn_save_table1] == 1:
            self.pub_save.publish(String(data='table1'))
            self.get_logger().info("SIMPAN: table1")
        if b[self.btn_go_table1] == 1:
            self.pub_go.publish(String(data='table1'))
            self.get_logger().info("PERGI: table1")
        if b[self.btn_save_table2] == 1:
            self.pub_save.publish(String(data='table2'))
            self.get_logger().info("SIMPAN: table2")
        if b[self.btn_go_table2] == 1:
            self.pub_go.publish(String(data='table2'))
            self.get_logger().info("PERGI: table2")


def main():
    rclpy.init()
    node = JoystickPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()