#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import numpy as np

class Bug0Node(Node):
    def __init__(self):
        super().__init__('bug0_node')

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz control loop

        # Goal parameters
        self.goal_x = -6.0
        self.goal_y = 3.0

        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0

        self.obstacle_detected = False
        self.distance_to_goal_threshold = 0.1
        self.obstacle_range_threshold = 0.5

        self.get_logger().info('Bug0 node initialized.')

    def odom_callback(self, msg):
        """Update current position and orientation from odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def scan_callback(self, msg):
        """Check for front-facing obstacles."""
        ranges = msg.ranges
        front_ranges = ranges[0:45] + ranges[315:360]
        min_dist = min([r for r in front_ranges if r > 0.01], default=float('inf'))
        self.obstacle_detected = min_dist < self.obstacle_range_threshold

    def get_direction_to_goal(self):
        """Calculate angle to the goal relative to current yaw."""
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.goal_y
        desired_angle = math.atan2(dy, dx)
        error = desired_angle - self.yaw

        return math.atan2(math.sin(error), math.cos(error))  # Normalize angle

    def control_loop(self):
        twist = Twist()

        if self.reached_goal():
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Reached goal!")
        elif self.obstacle_detected:
            # Follow wall on right-hand side by turning right
            twist.linear.x = 0.1
            twist.angular.z = -0.5
            self.get_logger().info("Obstacle detected: Following wall.")
        else:
            # Move towards goal
            angle_to_goal = self.get_direction_to_goal()
            twist.linear.x = 0.2
            twist.angular.z = max(-0.5, min(0.5, angle_to_goal * 1.5))
            self.get_logger().info(f"Moving toward goal. Angle error: {math.degrees(angle_to_goal):.2f}°")

        self.cmd_pub.publish(twist)

    def reached_goal(self):
        distance = math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)
        return distance < self.distance_to_goal_threshold


def main(args=None):
    rclpy.init(args=args)
    bug0_node = Bug0Node()
    try:
        rclpy.spin(bug0_node)
    except KeyboardInterrupt:
        bug0_node.get_logger().info("Shutting down Bug0 node.")
    finally:
        bug0_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()