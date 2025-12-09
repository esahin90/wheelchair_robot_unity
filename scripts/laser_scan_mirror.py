#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserScanMirror(Node):
    def __init__(self):
        super().__init__('laser_scan_mirror')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_flipped')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(
            LaserScan, input_topic, self.callback, 10)

        self.pub = self.create_publisher(LaserScan, output_topic, 10)

    def callback(self, msg: LaserScan):
        out = LaserScan()
        out = msg

        # Flip angles
        out.angle_min = -msg.angle_max
        out.angle_max = -msg.angle_min
        out.angle_increment = -msg.angle_increment

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanMirror()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
