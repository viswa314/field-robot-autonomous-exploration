#!/usr/bin/env python3

"""
Task 1: Control System - Circular Trajectory Controller
This node demonstrates pose-to-pose navigation and circular trajectory tracking.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import time


class CircularTrajectoryController(Node):
    def __init__(self):
        super().__init__('circular_trajectory_controller')

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False

        # Control parameters
        self.kp_linear = 0.45  # Proportional gain for linear velocity
        self.kp_angular = 0.9  # Proportional gain for angular velocity
        self.goal_tolerance = 0.10  # meters
        self.angle_tolerance = 0.1  # radians

        # Circular trajectory parameters
        self.circle_radius = 1.0  # meters
        self.circle_center_x = 0.0
        self.circle_center_y = 0.0
        self.angular_velocity = 0.2  # rad/s for circular motion

        self.get_logger().info('Circular Trajectory Controller initialized')
        self.get_logger().info('Waiting for odometry data...')

    def odom_callback(self, msg):
        """Update current robot pose from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to euler angles
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                            orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.current_theta = yaw

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('Odometry data received!')

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def move_to_pose(self, target_x, target_y, target_theta=None):
        """
        Task 1: Pose-to-pose navigation
        Move robot to target pose using proportional control
        """
        self.get_logger().info(
            f'Moving to pose: ({target_x:.2f}, {target_y:.2f})')

        rate = self.create_rate(20)  # 20 Hz control loop

        while rclpy.ok():
            # Calculate distance and angle to goal
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < self.goal_tolerance:
                # Goal reached
                self.stop_robot()
                self.get_logger().info('Pose reached!')

                # Orient to target heading if specified
                if target_theta is not None:
                    self.orient_to_heading(target_theta)

                return True

            # Calculate desired heading
            desired_theta = math.atan2(dy, dx)
            angle_error = self.normalize_angle(
                desired_theta - self.current_theta)

            # Create control command
            twist = Twist()

            # If angle error is large, rotate first
            if abs(angle_error) > 0.2:
                twist.linear.x = 0.0
                twist.angular.z = self.kp_angular * angle_error
            else:
                # Move forward while correcting heading
                twist.linear.x = min(self.kp_linear * distance, 0.5)
                twist.angular.z = self.kp_angular * angle_error

            self.cmd_vel_pub.publish(twist)

            # Log progress
            if int(time.time() * 2) % 10 == 0:  # Log every 5 seconds
                self.get_logger().info(
                    f'Distance: {distance:.2f}m, Angle error: {angle_error:.2f}rad'
                )

            rclpy.spin_once(self, timeout_sec=0.05)

        return False

    def orient_to_heading(self, target_theta):
        """Orient robot to specific heading"""
        self.get_logger().info(f'Orienting to heading: {target_theta:.2f} rad')

        rate = self.create_rate(20)

        while rclpy.ok():
            angle_error = self.normalize_angle(
                target_theta - self.current_theta)

            if abs(angle_error) < self.angle_tolerance:
                self.stop_robot()
                self.get_logger().info('Orientation reached!')
                return True

            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = self.kp_angular * angle_error
            self.cmd_vel_pub.publish(twist)

            rclpy.spin_once(self, timeout_sec=0.05)

        return False

    def track_circular_trajectory(self, duration=30.0):
        """
        Task 1: Circular trajectory tracking
        Robot follows a circular path for specified duration
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Starting circular trajectory tracking')
        self.get_logger().info(
            f'Radius: {self.circle_radius}m, Duration: {duration}s')
        self.get_logger().info('=' * 60)

        # First, move to starting position on circle
        start_x = self.circle_center_x + self.circle_radius
        start_y = self.circle_center_y
        self.move_to_pose(start_x, start_y)

        # Track circular trajectory
        start_time = time.time()
        rate = self.create_rate(20)

        while rclpy.ok() and (time.time() - start_time) < duration:
            # Calculate tangent velocity for circular motion
            # v = ω × r (linear velocity = angular velocity × radius)
            linear_vel = self.angular_velocity * self.circle_radius

            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = self.angular_velocity

            self.cmd_vel_pub.publish(twist)

            # Log progress every 2 seconds
            elapsed = time.time() - start_time
            if int(elapsed * 2) % 4 == 0:
                self.get_logger().info(
                    f'Tracking circle... {elapsed:.1f}/{duration}s elapsed'
                )

            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop_robot()
        self.get_logger().info('Circular trajectory completed!')

    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.5)

    def run_demo(self):
        """
        Task 1 Demo: Shows both pose-to-pose navigation and circular tracking
        """
        # Wait for odometry
        while not self.odom_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        self.get_logger().info('=' * 60)
        self.get_logger().info('TASK 1 DEMONSTRATION: CONTROL SYSTEM')
        self.get_logger().info('=' * 60)

        # Demo waypoints for pose-to-pose navigation
        waypoints = [
            (1.0, 0.0, 0.0),      # Point 1
            (1.0, 1.0, 1.57),     # Point 2
            (0.0, 1.0, 3.14),     # Point 3
            (0.0, 0.0, -1.57),    # Point 4 (back to start)
        ]

        self.get_logger().info('\nPart 1: Pose-to-Pose Navigation')
        self.get_logger().info(
            f'Navigating through {len(waypoints)} waypoints...\n')

        for i, (x, y, theta) in enumerate(waypoints, 1):
            self.get_logger().info(f'Waypoint {i}/{len(waypoints)}:')
            success = self.move_to_pose(x, y, theta)
            if not success:
                return
            time.sleep(1.0)  # Pause between waypoints

        self.get_logger().info('\nPart 2: Circular Trajectory Tracking')
        self.track_circular_trajectory(duration=60.0)

        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('TASK 1 COMPLETED SUCCESSFULLY!')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)

    controller = CircularTrajectoryController()

    try:
        controller.run_demo()
    except KeyboardInterrupt:
        controller.get_logger().info('Demo interrupted by user')
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
