import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        self.pose = None
        self.laser_data = None

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.pose = (pos.x, pos.y, yaw)
        self.get_logger().info(f"Current Pose : X : {pos.x}, Y : {pos.y}, Yaw : {yaw}")

    def laser_callback(self, msg):
        self.laser_data = msg.ranges

    def obstacle_detected(self, threshold=1):
        if not self.laser_data:
            return False
        center_range = self.laser_data[len(self.laser_data)//2]
        return center_range < threshold

    def find_clear_direction(self):
        if not self.laser_data:
            return 0.5  # Default left
        left = min(self.laser_data[len(self.laser_data)//2 + 30:])
        right = min(self.laser_data[:len(self.laser_data)//2 - 30])
        return 0.5 if left > right else -0.5

    def reorient_toward_goal(self, goal_x, goal_y):
        while rclpy.ok() and self.pose:
            dx = goal_x - self.pose[0]
            dy = goal_y - self.pose[1]
            target_yaw = math.atan2(dy, dx)
            yaw_error = target_yaw - self.pose[2]
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            self.get_logger().info(f"Error in orientation : {abs(yaw_error)}")

            if abs(yaw_error) < 0.5:
                break

            twist = Twist()
            twist.angular.z = 0.8 * yaw_error
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.cmd_pub.publish(Twist())

    def distance_to_goal(self, goal_x, goal_y):
        dx = goal_x - self.pose[0]
        dy = goal_y - self.pose[1]
        return math.sqrt(dx ** 2 + dy ** 2)

    def navigate_to_goal(self, goal_x, goal_y, goal_theta):
        self.reorient_toward_goal(goal_x, goal_y)
        self.get_logger().info("Starting navigation...")

        while rclpy.ok() and self.pose:
            if self.distance_to_goal(goal_x, goal_y) < 0.1:
                self.get_logger().info("Reached goal position.")
                break

            if self.obstacle_detected():
                self.get_logger().warn("Obstacle detected! Avoiding...")
                twist = Twist()
                twist.angular.z = self.find_clear_direction()
                self.cmd_pub.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.2)
                self.cmd_pub.publish(Twist())

                self.get_logger().info("Re-orienting after avoidance...")
                self.reorient_toward_goal(goal_x, goal_y)
                continue

            # Move toward goal
            twist = Twist()
            twist.linear.x = 0.5
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.cmd_pub.publish(Twist())
        self.align_with_theta(goal_theta)
        self.get_logger().info(f"Current Pose : X : {pos.x}, Y : {pos.y}, Yaw : {yaw}")
        self.get_logger().info(f"Goal pose : X : {goal_x}, Y : {goal_y}, Yaw : {math.radians(goal_theta)}")
        self.get_logger().info("Navigation complete.")

    def align_with_theta(self, target_theta):
        self.get_logger().info("Aligning with target orientation...")
        while rclpy.ok() and self.pose:
            yaw_error = target_theta - self.pose[2]
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            self.get_logger().info(f"Error in alignment : {abs(yaw_error)}")
            if abs(yaw_error) < 0.5:
                break
            twist = Twist()
            twist.angular.z = 0.4 * yaw_error
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    nav = Navigation()
    rclpy.spin_once(nav, timeout_sec=1.0)  # Allow subs to initialize

    
    nav.navigate_to_goal(goal_x=2.0, goal_y=-3.0, goal_theta=math.radians(90))

    nav.destroy_node()
    rclpy.shutdown()

