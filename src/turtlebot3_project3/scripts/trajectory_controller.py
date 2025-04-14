#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory
import os


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # Tuning parameters
        self.linear_kp = 2.0
        self.angular_kp = 1.0
        self.goal_tolerance = 0.05  # meters
        self.angle_tolerance = math.radians(10)  # radians

        # Load waypoints from file
        

        package_share = get_package_share_directory('turtlebot3_project3')
        waypoint_file = os.path.join(package_share, 'config', 'planner_waypoints.txt')
        self.waypoints = self.load_waypoints_from_file(waypoint_file)

        if not self.waypoints:
            self.get_logger().warn("No waypoints loaded. Shutting down.")
            rclpy.shutdown()
            return

        self.current_index = 0
        self.reached_position = False

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # in radians

        # Set up publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Waypoint Navigator node has started.")

    def load_waypoints_from_file(self, filename):
        waypoints = []
        try:
            with open(filename, 'r') as file:
                for line in file:
                    parts = line.strip().split(',')
                    if len(parts) == 3:
                        x, y, theta = map(float, parts)
                        waypoints.append((x, y, theta))
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints from file: {e}")
        return waypoints

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def control_loop(self):
        if self.current_index >= len(self.waypoints):
            self.stop_robot()
            self.get_logger().info("All waypoints reached.")
            return

        goal_x, goal_y, goal_theta_deg = self.waypoints[self.current_index]
        goal_theta_rad = math.radians(goal_theta_deg)

        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = math.hypot(dx, dy)
        heading_to_goal = math.atan2(dy, dx)
        heading_error = self.normalize_angle(heading_to_goal - self.yaw)

        self.get_logger().info(
            f"[Waypoint {self.current_index + 1}/{len(self.waypoints)}] "
            f"Current: x={self.x:.2f}, y={self.y:.2f}, yaw={math.degrees(self.yaw):.1f}°, "
            f"Distance: {distance:.3f} m, Heading Error: {math.degrees(heading_error):.1f}°"
        )

        cmd = Twist()

        # Phase 1: Go to waypoint position
        if not self.reached_position:
            if distance > self.goal_tolerance:
                if abs(heading_error) < math.radians(20):
                    cmd.linear.x = self.linear_kp * distance
                cmd.angular.z = self.angular_kp * heading_error
            else:
                self.get_logger().info(f"Reached position of waypoint {self.current_index + 1}")
                self.reached_position = True
        else:
            # Phase 2: Rotate to final heading
            final_yaw_error = self.normalize_angle(goal_theta_rad - self.yaw)
            if abs(final_yaw_error) > self.angle_tolerance:
                cmd.angular.z = self.angular_kp * final_yaw_error
            else:
                self.get_logger().info(f"Waypoint {self.current_index + 1} completed (position + heading)")
                self.current_index += 1
                self.reached_position = False

        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("Robot stopped.")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
