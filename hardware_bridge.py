#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist


class HardwareBridge(Node):

    def __init__(self):
        super().__init__('hardware_bridge')
        self.left_ticks = 0
        self.right_ticks = 0
        self.current_yaw = 0.0
        self.drive_state = 0
        try:
            self.serial_port = serial.Serial(
                '/dev/ttyUSB0',
                115200,
                timeout=0.01
            )

            self.get_logger().info("Connected to ESP32")

        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        self.timer = self.create_timer(
            0.02,
            self.read_serial
        )
        self.last_command = None
        self.cmd_vel_sub = self.create_subscription(
        Twist,
        '/cmd_vel',
        self.cmd_vel_callback,
        10
        )
    def cmd_vel_callback(self, msg):

        command = 'S'

        if msg.linear.x > 0.05:

            if msg.angular.z > 0.2:
                command = 'L'

            elif msg.angular.z < -0.2:
                command = 'R'

            else:
                command = 'F'

        else:
            command = 'S'

        if command != self.last_command:
            self.serial_port.write(command.encode())
            self.last_command = command
            self.get_logger().info(f"Sent: {command}")
    def read_serial(self):

        while self.serial_port.in_waiting > 0:

            try:
                line = self.serial_port.readline().decode(
                    'utf-8',
                    errors='ignore'
                ).strip()

                if not line:
                    continue

                # Ignore startup/debug messages
                if not line.startswith("E:"):
                    continue

                data = line[2:].split(',')

                if len(data) != 4:
                    continue

                self.left_ticks = int(data[0])
                self.right_ticks = int(data[1])
                self.current_yaw = float(data[2])
                self.drive_state = int(data[3])

                self.get_logger().info(
                    f"Left={self.left_ticks}  "
                    f"Right={self.right_ticks}  "
                    f"Yaw={self.current_yaw:.2f}  "
                    f"State={self.drive_state}"
                )

            except Exception as e:
                self.get_logger().warning(
                    f"Parse Error: {e}"
                )


def main(args=None):

    rclpy.init(args=args)

    node = HardwareBridge()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()