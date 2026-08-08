#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WanderNode(Node):

    def __init__(self):
        super().__init__('wander_node')

        # Latest scan
        self.latest_scan = None

        # Safe distance (meters)
        self.safe_distance = 0.7

        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # 10 Hz control loop
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info("Wander Node Started")

    def scan_callback(self, msg):

        self.latest_scan = msg

    def control_loop(self):

        if self.latest_scan is None:
            return

        ranges = self.latest_scan.ranges

        # Front 40 degrees (approximately)
        front = ranges[0:20] + ranges[-20:]

        valid = [
            r for r in front
            if r > 0.10 and r < 10.0
        ]

        if len(valid):
            closest = min(valid)
        else:
            closest = 999.0

        cmd = Twist()

        if closest < self.safe_distance:

            self.get_logger().info(
                f"Obstacle: {closest:.2f} m"
            )

            cmd.linear.x = 0.0
            cmd.angular.z = 0.5

        else:

            cmd.linear.x = 0.2
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = WanderNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:

        stop = Twist()
        node.cmd_vel_pub.publish(stop)

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()