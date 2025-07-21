#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import numpy as np

class Bug1Node(Node):
    def __init__(self):
        super().__init__('bug1_node')

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz control loop

        # Goal parameters
        self.goal_x = 2.0
        self.goal_y = 2.2

        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0

        # Obstacle detection
        self.obstacle_range_threshold = 0.7
        self.front_ranges = []
        self.obstacle_detected = False  # <-- Add this line

        # Bug1 state
        self.state = 'GO_TO_GOAL'  # Can be 'GO_TO_GOAL', 'FOLLOW_WALL'
        self.hit_point = None      # Point where obstacle was first detected
        self.closest_point_to_goal = None  # Best point on wall to resume goal movement
        self.min_distance_to_goal = float('inf')
        self.boundary_points = []  # List of points while circumnavigating

        self.distance_to_goal_threshold = 0.2

        self.get_logger().info('Bug1 node initialized.')

    def odom_callback(self, msg):
        """Update current position and orientation from odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def scan_callback(self, msg):
        """Check for front-facing obstacles."""
        ranges = msg.ranges
        self.front_ranges = ranges[0:45] + ranges[315:360]
        min_dist = min([r for r in self.front_ranges if r > 0.01], default=float('inf'))
        self.obstacle_detected = min_dist < self.obstacle_range_threshold

    def get_direction_to_goal(self):
        """Calculate angle to the goal relative to current yaw."""
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        desired_angle = math.atan2(dy, dx)
        error = desired_angle - self.yaw
        return math.atan2(math.sin(error), math.cos(error))  # Normalize angle

    def distance_to_goal(self):
        return math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)

    def control_loop(self):
        twist = Twist()

        if self.reached_goal():
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Reached goal!")
        elif self.state == 'GO_TO_GOAL':
            angle_error = self.get_direction_to_goal()
            if self.obstacle_detected:
                self.get_logger().info("Obstacle detected. Switching to wall following.")
                self.state = 'FOLLOW_WALL'
                self.hit_point = (self.current_x, self.current_y)
                self.boundary_points = []
                self.min_distance_to_goal = float('inf')
                self.closest_point_to_goal = None
            else:
                # Move toward goal
                twist.linear.x = 0.2
                twist.angular.z = max(-0.5, min(0.5, angle_error * 1.5))
                self.get_logger().info(f"Moving toward goal. Angle error: {math.degrees(angle_error):.2f}°")
        elif self.state == 'FOLLOW_WALL':
            # Follow wall on right-hand side
            twist.linear.x = 0.1
            twist.angular.z = -0.4  # Turn right slightly

            # Record boundary points
            current_pos = (self.current_x, self.current_y)
            self.boundary_points.append(current_pos)

            # Check if we've returned to hit point (completed circumnavigation)
            dist_from_hit = math.hypot(self.hit_point[0] - self.current_x,
                                       self.hit_point[1] - self.current_y)
            if dist_from_hit < 0.4:
                self.get_logger().info("Completed circumnavigation of obstacle.")

                # Find closest point to goal on boundary
                for pt in self.boundary_points:
                    dist = math.hypot(pt[0] - self.goal_x, pt[1] - self.goal_y)
                    if dist < self.min_distance_to_goal:
                        self.min_distance_to_goal = dist
                        self.closest_point_to_goal = pt

                self.get_logger().info(f"Resuming motion from closest point: {self.closest_point_to_goal}")
                self.state = 'GO_TO_GOAL'

        self.cmd_pub.publish(twist)

    def reached_goal(self):
        distance = math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)
        return distance < self.distance_to_goal_threshold


def main(args=None):
    rclpy.init(args=args)
    bug1_node = Bug1Node()
    try:
        rclpy.spin(bug1_node)
    except KeyboardInterrupt:
        bug1_node.get_logger().info("Shutting down Bug1 node.")
    finally:
        bug1_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()