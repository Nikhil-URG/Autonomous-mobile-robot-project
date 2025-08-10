#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
import tf_transformations
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_ros import TransformException
import cv2
from cv_bridge import CvBridge

class OccupancyGridMappingNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_mapper')

        # Map parameters
        self.map_size = 20.0   # meters
        self.resolution = 0.1  # meters per cell
        self.width = int(self.map_size / self.resolution)
        self.height = int(self.map_size / self.resolution)

        # Log-odds parameters
        self.P_occ = 0.1
        self.P_free = 0.8
        self.threshold_p_free = 0.7
        self.threshold_p_occ = 0.4
        self.l_occ = math.log(self.P_occ / (1 - self.P_occ))
        self.l_free = math.log(self.P_free / (1 - self.P_free))
        self.l0 = 0.0  # Prior log-odds = log(0.5/0.5)

        # Log-odds grid initialization
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Laser scan subscriber
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Occupancy grid publisher
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # Timer to publish the map regularly
        self.timer = self.create_timer(1.0, self.publish_map)

        self.bridge = CvBridge()

        # Motion filtering (update only if robot has moved enough)
        self.last_pose = None
        self.translation_threshold = 0.1  # meters
        self.rotation_threshold = math.radians(10)  # radians

        self.get_logger().info("Occupancy Grid Mapping Node started")

    def scan_callback(self, scan_msg):
        try:
            # Get robot pose from odom → base_link transform
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'odom', 
                'base_link', 
                now, 
                timeout=rclpy.duration.Duration(seconds=1)
            )


            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y

            q = trans.transform.rotation
            _, _, robot_theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

            # Motion threshold check
            if self.last_pose is not None:
                dx = robot_x - self.last_pose[0]
                dy = robot_y - self.last_pose[1]
                dtheta = robot_theta - self.last_pose[2]

                if math.hypot(dx, dy) < self.translation_threshold and abs(dtheta) < self.rotation_threshold:
                    return  # Skip update if robot hasn't moved enough

            self.last_pose = (robot_x, robot_y, robot_theta)

            angle = scan_msg.angle_min
            for r in scan_msg.ranges:
                if math.isinf(r) or math.isnan(r):
                    angle += scan_msg.angle_increment
                    continue

                end_x = robot_x + r * math.cos(robot_theta + angle)
                end_y = robot_y + r * math.sin(robot_theta + angle)

                self.update_map(robot_x, robot_y, end_x, end_y)

                angle += scan_msg.angle_increment

        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed (odom → base_link): {e}")

    def update_map(self, x0, y0, x1, y1):
        # Convert world coordinates to map grid coordinates
        def to_map_coords(x, y):
            mx = int((x + self.map_size / 2) / self.resolution)
            my = int((y + self.map_size / 2) / self.resolution)
            return mx, my

        mx0, my0 = to_map_coords(x0, y0)
        mx1, my1 = to_map_coords(x1, y1)

        # Ray tracing using Bresenham's algorithm
        points = self.bresenham(mx0, my0, mx1, my1)
        for cell in points[:-1]:  # Free space
            x, y = cell
            if 0 <= x < self.width and 0 <= y < self.height:
                self.log_odds[y][x] += self.l_free
                self.log_odds[y][x] = max(self.log_odds[y][x], math.log(0.01 / 0.99))  # Clamp min log-odds

        # Mark endpoint as occupied
        x, y = points[-1]
        if 0 <= x < self.width and 0 <= y < self.height:
            self.log_odds[y][x] += self.l_occ
            self.log_odds[y][x] = min(self.log_odds[y][x], math.log(0.99 / 0.01))  # Clamp max log-odds

    def bresenham(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for ray tracing"""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return cells

    def publish_map(self):
        occ_grid = OccupancyGrid()
        occ_grid.header.stamp = self.get_clock().now().to_msg()
        occ_grid.header.frame_id = 'odom'  # Changed from 'map' → 'odom'

        occ_grid.info.resolution = self.resolution
        occ_grid.info.width = self.width
        occ_grid.info.height = self.height
        occ_grid.info.origin.position.x = -self.map_size / 2
        occ_grid.info.origin.position.y = -self.map_size / 2
        occ_grid.info.origin.position.z = 0.0
        occ_grid.info.origin.orientation.w = 1.0

        flat_data = []
        for y in range(self.height):
            for x in range(self.width):
                l = self.log_odds[y][x]
                p = 1 - 1 / (1 + math.exp(l))
                if p > self.threshold_p_free:
                    flat_data.append(0)     # Free space
                elif p < self.threshold_p_occ:
                    flat_data.append(100)   # Occupied
                else:
                    flat_data.append(-1)    # Unknown

        occ_grid.data = flat_data
        self.map_pub.publish(occ_grid)

    def save_map_image(self):
        img = np.zeros((self.height, self.width), dtype=np.uint8)
        for y in range(self.height):
            for x in range(self.width):
                l = self.log_odds[y][x]
                p = 1 - 1 / (1 + math.exp(l))
                if p > self.threshold_p_free:
                    img[y][x] = 254  # White (free)
                elif p < self.threshold_p_occ:
                    img[y][x] = 0    # Black (occupied)
                else:
                    img[y][x] = 127  # Gray (unknown)

        img = cv2.flip(img, 0)
        cv2.imwrite('/home/lucifer/ros2_ws/src/amr_assign6/amr_assign6/occupancy_map.png', img)
        self.get_logger().info("Map image saved as occupancy_map.png")

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridMappingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down and saving map...")
        node.save_map_image()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
