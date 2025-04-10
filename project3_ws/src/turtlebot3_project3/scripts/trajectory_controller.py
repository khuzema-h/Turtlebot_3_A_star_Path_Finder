#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
import os
from pathlib import Path


class Trajectory_Controller(Node):
    def __init__(self):
        super().__init__("trajectory_controller")

        # Initialize publishers and subscribers
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # Publish control commands
        cmd_vel = Twist()

        # Define RPMs

        # We convert the RPM to Linear Velocity to publish to the /cmd_vel linear and angular topics

        wheel_radius = 0.033
        RPM_1 = ((2 * math.pi * 50) / 60) * wheel_radius
        RPM_2 = ((2 * math.pi * 100) / 60) * wheel_radius

        # Define the time duration for each move (seconds)
        # tf = float(input("Enter the time step duration: "))
        tf = 2

        # Read action list from file
        action_list = []
        try:
            # Get directory where this script is located
            script_dir = Path(os.path.dirname(os.path.realpath(__file__)))
            # Construct full path to action_list.txt
            file_path = script_dir / "action_list.txt"

            with open(file_path, "r") as file:
                for line in file:
                    line = line.strip()
                    if line:
                        # Remove parentheses and split by comma
                        values = line.strip("()").split(",")
                        if len(values) == 2:
                            try:
                                x = int(values[0])
                                y = int(values[1])
                                action_list.append((x, y))
                            except ValueError:
                                self.get_logger().warn(f"Skipping invalid line: {line}")
        except FileNotFoundError:
            self.get_logger().error(f"action_list.txt not found at {file_path}.")

        def forward_slow(tf):
            cmd_vel.linear.x = RPM_1
            self.get_logger().info("Moving Forward(slow)")
            self.pub.publish(cmd_vel)
            time.sleep(tf)
            cmd_vel.linear.x = 0.0
            self.pub.publish(cmd_vel)

        def right_slow(tf):
            cmd_vel.angular.z = -RPM_1
            self.get_logger().info("Turning Right (slow)")
            self.pub.publish(cmd_vel)
            time.sleep(tf)
            cmd_vel.angular.z = 0.0
            self.pub.publish(cmd_vel)

        def left_slow(tf):
            cmd_vel.angular.z = RPM_1
            self.get_logger().info("Turning Left (slow)")
            self.pub.publish(cmd_vel)
            time.sleep(tf)
            cmd_vel.angular.z = 0.0
            self.pub.publish(cmd_vel)

        def forward_fast(tf):
            cmd_vel.linear.x = RPM_2
            self.get_logger().info("Moving Forward (Fast)")
            self.pub.publish(cmd_vel)
            time.sleep(tf)
            cmd_vel.linear.x = 0.0
            self.pub.publish(cmd_vel)

        def right_fast(tf):
            cmd_vel.angular.z = -RPM_2
            self.get_logger().info("Turning Right (Fast)")
            self.pub.publish(cmd_vel)
            time.sleep(tf)
            cmd_vel.angular.z = 0.0
            self.pub.publish(cmd_vel)

        def right_forward(tf):
            cmd_vel.angular.z = -RPM_1
            cmd_vel.linear.x = RPM_2
            self.get_logger().info("Turning Right and Forward")
            self.pub.publish(cmd_vel)
            time.sleep(tf)
            cmd_vel.angular.z = 0.0
            cmd_vel.linear.x = 0.0
            self.pub.publish(cmd_vel)

        def left_forward(tf):
            cmd_vel.angular.z = RPM_1
            cmd_vel.linear.x = RPM_2
            self.get_logger().info("Turning Left and Forward")
            self.pub.publish(cmd_vel)
            time.sleep(tf)
            cmd_vel.angular.z = 0.0
            cmd_vel.linear.x = 0.0
            self.pub.publish(cmd_vel)

        def left_fast(tf):
            cmd_vel.angular.z = RPM_2
            self.get_logger().info("Turning Left (Fast)")
            self.pub.publish(cmd_vel)
            time.sleep(tf)
            cmd_vel.angular.z = 0.0
            self.pub.publish(cmd_vel)

        def stop():
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.get_logger().info("Stopping")
            self.pub.publish(cmd_vel)

        for action in action_list:
            if action == (0, 50):
                left_slow(tf)
            elif action == (50, 0):
                right_slow(tf)

            elif action == (50, 50):
                forward_slow(tf)

            elif action == (0, 100):
                left_fast(tf)

            elif action == (100, 0):
                right_fast(tf)

            elif action == (100, 100):
                forward_fast(tf)
            elif action == (50, 100):
                left_forward(tf)

            elif action == (100, 50):
                right_forward(tf)
        stop()
        
        


def main(args=None):
    rclpy.init(args=args)
    controller = Trajectory_Controller()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
