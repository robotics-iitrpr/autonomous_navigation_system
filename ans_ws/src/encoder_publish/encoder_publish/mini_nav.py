#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from tf_transformations import euler_from_quaternion
from queue import PriorityQueue


class MiniNavNode(Node):
    def __init__(self):
        super().__init__('mini_nav')

        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Internal state
        self.map_data = None
        self.odom = None
        self.goal = None
        self.grid = None
        self.resolution = None
        self.origin = None
        self.path = []
        self.path_index = 0

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        self.grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.get_logger().info("Map received")

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def goal_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.goal = self.world_to_map(x, y)
        self.path = []
        self.path_index = 0
        self.get_logger().info(f"New goal received: {self.goal}")

    def world_to_map(self, x, y):
        mx = int((x - self.origin[0]) / self.resolution)
        my = int((y - self.origin[1]) / self.resolution)
        return mx, my

    def map_to_world(self, mx, my):
        x = mx * self.resolution + self.origin[0] + self.resolution / 2
        y = my * self.resolution + self.origin[1] + self.resolution / 2
        return x, y

    def is_occupied(self, x, y):
        try:
            return self.grid[y][x] > 50  # threshold
        except IndexError:
            return True

    def a_star(self, start, goal):
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            _, current = frontier.get()

            if current == goal:
                break

            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                next_node = (current[0] + dx, current[1] + dy)

                if 0 <= next_node[0] < self.grid.shape[1] and 0 <= next_node[1] < self.grid.shape[0]:
                    if self.is_occupied(*next_node):
                        continue
                    new_cost = cost_so_far[current] + 1
                    if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                        cost_so_far[next_node] = new_cost
                        priority = new_cost + self.heuristic(goal, next_node)
                        frontier.put((priority, next_node))
                        came_from[next_node] = current

        # Reconstruct path
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from.get(current)
            if current is None:
                return []  # No path
        path.reverse()
        return path

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def control_loop(self):
        if self.map_data is None or self.odom is None or self.goal is None:
            return

        # Get robot position
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        mx, my = self.world_to_map(x, y)

        if not self.path:
            self.path = self.a_star((mx, my), self.goal)
            self.path_index = 0
            if not self.path:
                self.get_logger().warn("Path planning failed!")
                return
            self.get_logger().info(f"Path found with {len(self.path)} points.")
            self.publish_path()

        # Reached end of path
        if self.path_index >= len(self.path):
            self.get_logger().info("Goal reached.")
            self.cmd_pub.publish(Twist())  # Stop
            return

        # Follow path
        tx, ty = self.map_to_world(*self.path[self.path_index])
        dx = tx - x
        dy = ty - y
        distance = math.hypot(dx, dy)

        # Get yaw from odometry
        orientation_q = self.odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - yaw)

        # Control logic
        cmd = Twist()
        if distance > 0.1:
            # Scaled speed control
            cmd.linear.x = 0.2
            cmd.angular.z = 1.0 * angle_diff
        else:
            self.path_index += 1

        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for cell in self.path:
            wx, wy = self.map_to_world(*cell)
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0  # No rotation
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MiniNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
