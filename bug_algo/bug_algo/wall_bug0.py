#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import numpy as np

class Bug0Node(Node):
    def __init__(self):
        super().__init__('bug0_node')

        # Declare ROS parameters for configurability
        self.declare_parameter('goal_x', -6.0)
        self.declare_parameter('goal_y', 3.0)
        self.declare_parameter('distance_to_goal_threshold', 0.1)
        self.declare_parameter('obstacle_range_threshold', 0.5)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('laser_angle_range', 45.0)  # Degrees for front-facing scan

        # Get parameter values
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.distance_to_goal_threshold = self.get_parameter('distance_to_goal_threshold').value
        self.obstacle_range_threshold = self.get_parameter('obstacle_range_threshold').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.laser_angle_range = self.get_parameter('laser_angle_range').value

        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0
        self.obstacle_detected = False
        self.state = 'MOVE_TO_GOAL'  # States: MOVE_TO_GOAL, FOLLOW_WALL

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'Bug0 node initialized with goal ({self.goal_x}, {self.goal_y})')

    def odom_callback(self, msg):
        """Update current position and orientation from odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def scan_callback(self, msg):
        """Check for obstacles in the front-facing laser scan."""
        ranges = msg.ranges
        angle_increment = msg.angle_increment * 180.0 / math.pi  # Convert to degrees
        front_angle = self.laser_angle_range  # Configurable front angle range

        # Calculate indices for front-facing scan (e.g., -45° to +45°)
        num_front_indices = int(front_angle / angle_increment)
        center_idx = len(ranges) // 2
        front_indices = list(range(center_idx - num_front_indices, center_idx + num_front_indices + 1))
        front_ranges = [ranges[i] for i in front_indices if 0 < i < len(ranges) and ranges[i] > 0.01]

        # Check for obstacle within threshold
        min_dist = min(front_ranges, default=float('inf'))
        self.obstacle_detected = min_dist < self.obstacle_range_threshold

    def get_distance_to_goal(self):
        """Calculate Euclidean distance to the goal."""
        return math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)

    def get_direction_to_goal(self):
        """Calculate angle to the goal relative to current yaw."""
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        desired_angle = math.atan2(dy, dx)
        error = desired_angle - self.yaw
        return math.atan2(math.sin(error), math.cos(error))  # Normalize angle

    def can_move_to_goal(self):
        """Check if the robot can move directly toward the goal (no obstacle and path is clear)."""
        return not self.obstacle_detected

    def move_to_goal(self):
        """Generate velocity command to move toward the goal."""
        twist = Twist()
        angle_to_goal = self.get_direction_to_goal()
        twist.linear.x = self.linear_speed
        twist.angularzonego.angular.z = max(-self.angular_speed, min(self.angular_speed, angle_to_goal * 1.5))
        return twist

    def follow_wall(self):
        """Generate velocity command to follow the obstacle's boundary (left side)."""
        twist = Twist()
        twist.linear.x = self.linear_speed * 0.5  # Slower speed for wall-following
        twist.angular.z = self.angular_speed  # Positive for left turn (counterclockwise)
        return twist

    def control_loop(self):
        """Main control loop implementing Bug0 algorithm."""
        twist = Twist()

        # Check if goal is reached
        if self.get_distance_to_goal() < self.distance_to_goal_threshold:
            self.state = 'MOVE_TO_GOAL'  # Reset state
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Reached goal!")
        else:
            # State machine for Bug0 algorithm
            if self.state == 'MOVE_TO_GOAL':
                if self.can_move_to_goal():
                    twist = self.move_to_goal()
                    self.get_logger().debug(f"Moving to goal. Angle error: {math.degrees(self.get_direction_to_goal()):.2f}°")
                else:
                    self.state = 'FOLLOW_WALL'
                    twist = self.follow_wall()
                    self.get_logger().debug("Obstacle detected: Switching to wall-following.")
            elif self.state == 'FOLLOW_WALL':
                if self.can_move_to_goal():
                    self.state = 'MOVE_TO_GOAL'
                    twist = self.move_to_goal()
                    self.get_logger().debug("Path clear: Switching to move-to-goal.")
                else:
                    twist = self.follow_wall()
                    self.get_logger().debug("Following wall (left).")

        self.cmd_pub.publish(twist)

    def destroy_node(self):
        """Cleanup before shutting down."""
        twist = Twist()  # Stop the robot
        self.cmd_pub.publish(twist)
        super().destroy_node()

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