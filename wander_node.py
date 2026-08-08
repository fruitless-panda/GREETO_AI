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

    # ----------------------------
    # Define sectors
    # ----------------------------

    front = ranges[0:20] + ranges[-20:]
    left  = ranges[40:90]
    right = ranges[-90:-40]

    def average_distance(data):

        valid = [r for r in data if 0.10 < r < 10.0]

        if not valid:
            return 0.0

        return sum(valid) / len(valid)

        front_dist = average_distance(front)
        left_dist  = average_distance(left)
        right_dist = average_distance(right)

        cmd = Twist()

    # ----------------------------
    # State Machine
    # ----------------------------

        if self.turning:

            # Continue turning until front is clear

            if front_dist > self.safe_distance:

                self.turning = False

                self.get_logger().info("Path Clear")

                cmd.linear.x = 0.2
                cmd.angular.z = 0.0

            else:

                cmd.linear.x = 0.0

                if self.turn_direction == "LEFT":
                    cmd.angular.z = 0.5
                else:
                    cmd.angular.z = -0.5

        else:

            # Normal forward driving

            if front_dist > self.safe_distance:

                cmd.linear.x = 0.2
                cmd.angular.z = 0.0

            else:

                self.turning = True

                # Choose side with more free space

                if left_dist > right_dist:
                    self.turn_direction = "LEFT"
                    cmd.angular.z = 0.5
                    self.get_logger().info("Turning Left")
                else:
                    self.turn_direction = "RIGHT"
                    cmd.angular.z = -0.5
                    self.get_logger().info("Turning Right")

                cmd.linear.x = 0.0

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