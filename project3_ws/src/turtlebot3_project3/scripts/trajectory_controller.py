#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class Trajectory_Controller(Node):
    def __init__(self):
        super().__init__("trajectory_controller")

        # Initialize publishers and subscribers
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # Publish control commands
        cmd_vel = Twist()
        
        # Define RPMs
        RPM_1 = 0.5
        RPM_2 = 1.0
        
        

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

       
        tf = 2
        
        action_list = [(0,50),(50,0), (50,50), (0,100), (100,0), (100,100), (50,100), (100,50)]
        
        for action in action_list:
            if action == (0,50):
                left_slow(tf)
            elif action ==(50,0):
                right_slow(tf)
            
            elif action == (50,50):
                forward_slow(tf)
            
            elif action == (0,100):
                left_fast(tf)
            
            elif action == (100,0):
                right_fast(tf)
            
            elif action == (100,100):
                forward_fast(tf)
            elif action == (50,100):
                right_forward(tf)
            
            elif action == (100,50):
                left_forward(tf)
        stop()
        

def main(args=None):
    rclpy.init(args=args)
    controller = Trajectory_Controller()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
