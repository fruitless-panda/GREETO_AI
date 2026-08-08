#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class WanderNode(Node):

    def __init__(self):
        super().__init__('wander_node')

        # Publish movement commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Run control loop at 10 Hz
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info("Wander Node Started")

    def control_loop(self):

        cmd = Twist()

        # Drive forward slowly
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