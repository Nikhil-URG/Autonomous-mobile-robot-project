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
        # Create the publisher nodes for cmd vel topic
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Create the subscribers required for odom data as well as LaserScan
        self.odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser = self.create_subscription(LaserScan, '/laser_scan', self.laser_callback, 10)

        self.pose = None
        self.obstacle_detected = False

    def odom_callback(self, msg):
        # Store the current position from the data received by subscriber to odom

        curr_pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # The data is in the form of quaternion so converting it to euler

        temp1, temp2, theta = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        self.pose = (curr_pos.x, curr_pos.y, theta)
        self.get_logger().info(f"Current pose: x={curr_pos.x}, y={curr_pos.y}, theta={theta}")


    def laser_callback(self, msg):
        # We need to make sure there are not obstacles in the robots path
        min_dist = min(msg.ranges)
        # Setting the threshold for minimum distance as 0.5m
        

        self.obstacle_detected = min_dist < 1

    def orient_towards_goal(self, goal_x, goal_y):
        self.get_logger().info("Starting orientation towards goal...")
        while rclpy.ok() and self.pose:
            # Find the required change for rotation required to orient towards goal
            delta_x = goal_x - self.pose[0]
            delta_y = goal_y - self.pose[1]
            required_yaw = math.atan2(delta_y, delta_x)
            delta_theta = required_yaw - self.pose[2]
            self.get_logger().info(f"Error in orientation : {abs(delta_theta)}")

            # Account for error
            if abs(delta_theta) < 0.5:
                self.get_logger().info("Orientation towards goal complete.")
                break


            twist = Twist()
            twist.angular.z = 1 * delta_theta
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec = 0.2)

        self.cmd_pub.publish(Twist())  # Stop rotation when robot has reached proper orientation

    def move_robot_twd_goal(self, goal_x, goal_y):
        self.get_logger().info("Starting movement toward goal...")

        while rclpy.ok() and self.pose:
            dx = goal_x - self.pose[0]
            dy = goal_y - self.pose[1]
            dist = math.sqrt(dx**2 + dy**2)

            if dist < 0.1:
                break

            if self.obstacle_detected:
                # Try to go left whenever obstacle is detected
                self.get_logger().info("Obstacle has been detected now rotating!")
                twist = Twist()
                twist.angular.z = 0.5
            else:
                twist = Twist()
                twist.linear.x = 0.5
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.cmd_pub.publish(Twist())  # Stop

    def align_with_theta(self, target_theta):
        self.get_logger().info("Final orientation alignment...")
        while rclpy.ok() and self.pose:
            yaw_error = target_theta - self.pose[2]
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            self.get_logger().info(f"Error in orientation : {abs(yaw_error)}")

            if abs(yaw_error) < 0.05:
                self.get_logger().info("Finished orientation alignment!")
                break

            twist = Twist()
            twist.angular.z = 0.5 * yaw_error
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.cmd_pub.publish(Twist())  # Stop

    def specify_goal(self, x, y, theta):
        self.orient_towards_goal(x, y)
        self.move_robot_twd_goal(x, y)
        self.align_with_theta(theta)
        self.get_logger().info("Goal processing complete.")

def main(args=None):
    rclpy.init(args=args)
    navigator = Navigation()

    while rclpy.ok() and navigator.pose is None:
        rclpy.spin_once(navigator, timeout_sec=0.1)
        navigator.get_logger().info("Waiting for odometry...")

    # Replace with desired target
    navigator.specify_goal(5.0, 8.0, math.radians(90))

    navigator.destroy_node()
    rclpy.shutdown()








