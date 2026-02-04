#!/usr/bin/env python3

"""
Task 2 & 3: Autonomous Exploration Controller
Implements frontier-based exploration for unknown environments
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
import math
import time


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # QoS profile for map
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1
        )

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        # Action client for navigation
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # Map data
        self.map_data = None
        self.map_received = False

        # Exploration parameters
        self.min_frontier_size = 10  # Minimum frontier cluster size
        self.obstacle_distance = 5   # Grid cells to stay away from obstacles
        self.explored_frontiers = []

        self.get_logger().info('Frontier Explorer initialized')
        self.get_logger().info('Waiting for map data...')

    def map_callback(self, msg):
        """Process occupancy grid map"""
        self.map_data = msg
        if not self.map_received:
            self.map_received = True
            self.get_logger().info('Map data received!')

    def get_frontiers(self):
        """
        Task 2: Frontier Detection
        Identify frontiers (boundaries between known and unknown space)
        """
        if self.map_data is None:
            return []

        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin

        # Convert map data to 2D array
        grid = np.array(self.map_data.data).reshape((height, width))

        frontiers = []

        # Scan for frontier cells
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                # Check if cell is free space
                if grid[y, x] == 0:
                    # Check if any neighbor is unknown
                    neighbors = [
                        grid[y-1, x], grid[y+1, x],
                        grid[y, x-1], grid[y, x+1],
                        grid[y-1, x-1], grid[y-1, x+1],
                        grid[y+1, x-1], grid[y+1, x+1]
                    ]

                    if -1 in neighbors:  # -1 = unknown
                        # Check if far enough from obstacles
                        if self.is_safe_location(grid, x, y):
                            # Convert grid coordinates to world coordinates
                            world_x = origin.position.x + \
                                (x + 0.5) * resolution
                            world_y = origin.position.y + \
                                (y + 0.5) * resolution
                            frontiers.append((world_x, world_y))

        # Cluster frontiers
        clustered_frontiers = self.cluster_frontiers(frontiers)

        self.get_logger().info(
            f'Found {len(clustered_frontiers)} frontier clusters')
        return clustered_frontiers

    def is_safe_location(self, grid, x, y):
        """Check if location is safe (away from obstacles)"""
        height, width = grid.shape

        for dy in range(-self.obstacle_distance, self.obstacle_distance + 1):
            for dx in range(-self.obstacle_distance, self.obstacle_distance + 1):
                ny, nx = y + dy, x + dx
                if 0 <= ny < height and 0 <= nx < width:
                    if grid[ny, nx] > 50:  # Occupied cell
                        return False
        return True

    def cluster_frontiers(self, frontiers):
        """Cluster nearby frontier points and return centroids"""
        if not frontiers:
            return []

        clusters = []
        used = set()

        for i, point in enumerate(frontiers):
            if i in used:
                continue

            cluster = [point]
            used.add(i)

            # Find nearby points
            for j, other_point in enumerate(frontiers):
                if j in used:
                    continue

                dist = math.sqrt(
                    (point[0] - other_point[0])**2 +
                    (point[1] - other_point[1])**2
                )

                if dist < 0.5:  # 0.5 meter clustering threshold
                    cluster.append(other_point)
                    used.add(j)

            # Filter small clusters
            if len(cluster) >= self.min_frontier_size:
                # Calculate centroid
                centroid_x = sum(p[0] for p in cluster) / len(cluster)
                centroid_y = sum(p[1] for p in cluster) / len(cluster)
                clusters.append((centroid_x, centroid_y, len(cluster)))

        return clusters

    def select_best_frontier(self, frontiers, robot_x, robot_y):
        """
        Task 2: Frontier Selection
        Select best frontier based on distance and size
        """
        if not frontiers:
            return None

        best_frontier = None
        best_score = -float('inf')

        for fx, fy, size in frontiers:
            # Skip if already explored
            if self.is_frontier_explored(fx, fy):
                continue

            # Calculate distance
            distance = math.sqrt((fx - robot_x)**2 + (fy - robot_y)**2)

            # Score = size_weight * size - distance_weight * distance
            # Prefer larger frontiers that are closer
            score = 2.0 * size - 1.0 * distance

            if score > best_score:
                best_score = score
                best_frontier = (fx, fy)

        return best_frontier

    def is_frontier_explored(self, x, y):
        """Check if frontier was already explored"""
        for ex, ey in self.explored_frontiers:
            dist = math.sqrt((x - ex)**2 + (y - ey)**2)
            if dist < 1.0:  # 1 meter threshold
                return True
        return False

    def navigate_to_goal(self, x, y):
        """
        Task 3: Navigate to goal using Nav2
        """
        self.get_logger().info(f'Navigating to frontier: ({x:.2f}, {y:.2f})')

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        # Send goal
        self.get_logger().info('Sending navigation goal...')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(
            self, send_goal_future, timeout_sec=5.0)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return False

        self.get_logger().info('Goal accepted, navigating...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

        result = result_future.result()
        if result:
            self.get_logger().info('✓ Navigation succeeded!')
            self.explored_frontiers.append((x, y))
            return True
        else:
            self.get_logger().warn('Navigation failed or timed out')
            return False

    def explore(self):
        """
        Task 3: Main exploration loop
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('STARTING AUTONOMOUS EXPLORATION')
        self.get_logger().info('=' * 60)

        # Wait for map
        while not self.map_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        exploration_count = 0
        max_iterations = 20

        while rclpy.ok() and exploration_count < max_iterations:
            exploration_count += 1

            self.get_logger().info(
                f'\n--- Exploration Iteration {exploration_count} ---')

            # Find frontiers
            frontiers = self.get_frontiers()

            if not frontiers:
                self.get_logger().info('No more frontiers found!')
                break

            # Select best frontier (assume robot at origin for demo)
            best_frontier = self.select_best_frontier(frontiers, 0.0, 0.0)

            if best_frontier is None:
                self.get_logger().info('All frontiers explored!')
                break

            # Navigate to frontier
            success = self.navigate_to_goal(best_frontier[0], best_frontier[1])

            if not success:
                self.get_logger().warn('Navigation failed, trying next frontier')
                self.explored_frontiers.append(best_frontier)

            time.sleep(2.0)  # Brief pause between explorations

        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('✓ EXPLORATION COMPLETED!')
        self.get_logger().info(
            f'Explored {len(self.explored_frontiers)} frontiers')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)

    explorer = FrontierExplorer()

    try:
        explorer.explore()
    except KeyboardInterrupt:
        explorer.get_logger().info('Exploration interrupted by user')
    finally:
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
