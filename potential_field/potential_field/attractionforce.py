#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class PotentialFieldNode(Node):
    def __init__(self):
        super().__init__('potential_field_node')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Goal position (x, y) in meters
        self.goal_x = 2.0  # Example goal position
        self.goal_y = 2.0
        
        # Potential field parameters
        self.k_att = 0.5  # Attractive gain
        self.max_linear_speed = 0.22  # Max linear speed for TurtleBot3 Burger
        self.max_angular_speed = 2.84  # Max angular speed for TurtleBot3 Burger
        self.goal_threshold = 0.1  # Distance threshold to consider goal reached
        
        # Robot's current position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

    def odom_callback(self, msg):
        # Extract current position and orientation
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Compute attractive force
        self.compute_potential_field()

    def compute_potential_field(self):
        # Calculate distance to goal
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if goal is reached
        if distance < self.goal_threshold:
            self.get_logger().info('Goal reached!')
            self.stop_robot()
            return
        
        # Calculate angle to goal
        goal_angle = math.atan2(dy, dx)
        angle_diff = goal_angle - self.current_yaw
        
        # Normalize angle to [-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        
        # Attractive force: F_att = k_att * distance_vector
        force_x = self.k_att * dx
        force_y = self.k_att * dy
        
        # Convert force to velocity
        linear_vel = math.sqrt(force_x**2 + force_y**2)
        angular_vel = angle_diff
        
        # Cap velocities
        linear_vel = min(linear_vel, self.max_linear_speed)
        angular_vel = max(min(angular_vel, self.max_angular_speed), -self.max_angular_speed)
        
        # Publish velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.get_logger().info(f'Distance to goal: {distance:.2f}, Linear: {linear_vel:.2f}, Angular: {angular_vel:.2f}')

    def stop_robot(self):
        # Publish zero velocity to stop the robot
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
        node.get_logger().info('Node stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()