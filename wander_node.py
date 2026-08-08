#!/usr/bin/env python3
import math
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
        self.turning = False
        self.turn_direction = "LEFT"

    def scan_callback(self, msg):

        self.latest_scan = msg

    def average_distance(self, data):

        valid = [
            r for r in data
            if math.isfinite(r) and 0.10 < r < 10.0
        ]

        if not valid:
            return 0.0

        return sum(valid) / len(valid)
    def is_clear(self, data, clearance=0.5):

        valid = [
            r for r in data
            if math.isfinite(r) and 0.10 < r < 10.0
        ]

        if not valid:
            return False

        return min(valid) >= clearance
    def minimum_distance(self, data):

        valid = [
            r for r in data
            if math.isfinite(r) and 0.10 < r < 10.0
        ]

        if not valid:
            return 0.0

        return min(valid)

    def control_loop(self):

        if self.latest_scan is None:
            return

        ranges = self.latest_scan.ranges

        # -------------------------------------------------
        # Front semicircle
        #
        # 270° -> 360° -> 0° -> 90°
        #
        # Left side  = 270° -> 360°
        # Right side =   0° -> 90°
        # -------------------------------------------------

        left_side = ranges[377:503]
        right_side = ranges[0:125]

        front_semicircle = left_side + right_side

        # -------------------------------------------------
        # Check entire front semicircle
        # -------------------------------------------------

        semicircle_clear = self.is_clear(
            front_semicircle,
            0.5
        )

        # -------------------------------------------------
        # Calculate average clearance
        # -------------------------------------------------

        left_clearance = self.average_distance(
            left_side
        )

        right_clearance = self.average_distance(
            right_side
        )

        cmd = Twist()

        # -------------------------------------------------
        # TURNING
        # -------------------------------------------------

        if self.turning:

            # NEVER drive forward until the entire
            # front semicircle is clear.

            if semicircle_clear:

                self.turning = False

                self.get_logger().info(
                    "Front semicircle clear -> FORWARD"
                )

                cmd.linear.x = 0.2
                cmd.angular.z = 0.0

            else:

                # Obstacle still present.
                # Keep turning in the selected direction.

                cmd.linear.x = 0.0

                if self.turn_direction == "LEFT":

                    cmd.angular.z = 0.5

                else:

                    cmd.angular.z = -0.5

        # -------------------------------------------------
        # FORWARD
        # -------------------------------------------------

        else:

            if semicircle_clear:

                # Entire 180° front area is clear.

                cmd.linear.x = 0.2
                cmd.angular.z = 0.0

            else:

                # Something is inside the 50 cm
                # safety boundary.

                self.turning = True

                # Choose the side with the larger
                # average clearance.

                if left_clearance > right_clearance:

                    self.turn_direction = "LEFT"

                    self.get_logger().info(
                        f"OBSTACLE -> LEFT "
                        f"L={left_clearance:.2f} "
                        f"R={right_clearance:.2f}"
                    )

                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.5

                else:

                    self.turn_direction = "RIGHT"

                    self.get_logger().info(
                        f"OBSTACLE -> RIGHT "
                        f"L={left_clearance:.2f} "
                        f"R={right_clearance:.2f}"
                    )

                    cmd.linear.x = 0.0
                    cmd.angular.z = -0.5

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