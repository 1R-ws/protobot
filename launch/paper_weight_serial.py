#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial


class PaperWeightNode(Node):
    def __init__(self):
        super().__init__('paper_weight_node')

        # --------------------------------------------------
        # Parameters
        # --------------------------------------------------
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('publish_rate', 10.0)  # Hz

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # --------------------------------------------------
        # Serial Interface
        # --------------------------------------------------
        self.ser = serial.Serial(self.port, self.baud, timeout=1)

        # --------------------------------------------------
        # Publishers
        # --------------------------------------------------
        self.paper_weight_pub = self.create_publisher(
            Float32,
            '/paper_weight',
            10
        )

        # --------------------------------------------------
        # Timers
        # --------------------------------------------------
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.read_serial)

        self.get_logger().info(
            f"PaperWeightNode started | port={self.port}, baud={self.baud}"
        )

    # --------------------------------------------------
    # Serial Read Callback
    # --------------------------------------------------
    def read_serial(self):
        line = self.ser.readline().decode(errors='ignore').strip()

        if not line:
            return

        try:
            weight = float(line)

            msg = Float32()
            msg.data = weight
            self.paper_weight_pub.publish(msg)

        except ValueError:
            # Ignore non-numeric serial messages
            pass


def main():
    rclpy.init()
    node = PaperWeightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
