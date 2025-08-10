import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import tf_transformations

class MovementManager(Node):
    def __init__(self, goal_x, goal_y, goal_yaw):
        super().__init__("movement_node")
        # Defining thresholds here for final pose and orientation of the robot.
        # here we are considering the FOV of robot that is why we are taking angle into consideration.
        self.angle_threshold = 0.01
        self.position_threshold = 0.5
        self.mode = 'rotate'
        self.get_logger().warn("Starting to rotate towards goal")

        # Creating a position dictionary to store the current position from the odom topic

        self.position = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.goal = {"x": goal_x, "y": goal_y, "yaw": goal_yaw}

        # Creating subscribers for receiving laser and odom data.
        # Creating publishers for sending comamnds to cmd_vel to contol robile.

        self.odom_subscriber = self.create_subscription(Odometry, "/odom", self.odom_handler, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, "/scan", self.laser_handler, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.state_machine)

        self.path_clear = True
        # Maintaining a recovery counter to track robots attemts to recover from situations

        self.recovery_attempts = 0

    def odom_handler(self, data):
        # Callback function for handling the odometry data.
        self.position["x"] = data.pose.pose.position.x
        self.position["y"] = data.pose.pose.position.y
        # storing in a quaternion so later we can convert to euler only to get yaw values.
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
        self.position["yaw"] = yaw

    def laser_handler(self, scan):
        # callback function to handle laser data.
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        # making sure the FOV of robot is captured.
        valid_ranges = [r for r, a in zip(scan.ranges, angles) if abs(a) <= np.radians(30)]
        valid_ranges = [r for r in valid_ranges if not np.isnan(r)]
        # setting threshold as 0.5m from robot.
        if valid_ranges and min(valid_ranges) < 0.5:
            self.path_clear = False
        else:
            self.path_clear = True

    def rotate_to_goal(self):
        twist = Twist()
        desired_angle = np.arctan2(
            self.goal["y"] - self.position["y"],
            self.goal["x"] - self.position["x"]
        )
        angle_diff = desired_angle - self.position["yaw"]

        if abs(angle_diff) > self.angle_threshold:
            twist.angular.z = 1.0
            self.velocity_publisher.publish(twist)
            self.log_status("Adjusting orientation...")
        else:
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)
            self.mode = 'move'
            self.recovery_attempts = 0
            self.log_status("Facing goal direction.")
            self.get_logger().warn("Facing goal direction!")

    def move_to_goal(self):
        twist = Twist()
        # Use euclidean distance for estimating distance to goal.
        distance = np.sqrt(
            (self.goal["x"] - self.position["x"]) ** 2 +
            (self.goal["y"] - self.position["y"]) ** 2
        )
        if distance > self.position_threshold:
            if self.path_clear:
                twist.linear.x = 1.0
                self.velocity_publisher.publish(twist)
                self.log_status(f"Proceeding to goal, Distance to goal: {distance:.2f}")
                self.recovery_attempts = 0
            else:
                twist.linear.x = 0.0
                self.velocity_publisher.publish(twist)
                self.log_status("Obstacle has been encountered!")
                self.mode = 'recover'
        else:
            twist.linear.x = 0.0
            self.velocity_publisher.publish(twist)
            self.mode = 'adjust'
            self.log_status("Arrived near target.")
            self.get_logger().warn("Have reached the goal")

    def adjust_orientation(self):
        self.mode = 'idle'
        self.log_status("Reached goal and now final orientation at goal!")
        self.get_logger().warn("Reached goal and now final orientation at goal!")
    def recover(self):
        # Recovery code, whenever robot is encountering an obstacle.
        twist = Twist()
        if self.recovery_attempts < 10:
            # try 10 times to reorient and get out of locked position
            twist.angular.z = 0.5
            self.velocity_publisher.publish(twist)
            self.recovery_attempts += 1
            self.log_status(f"Executing recovery turn: attempt {self.recovery_attempts}.")
            self.get_logger.warn("Trying to recover robot!")
        else:
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)
            self.log_status("Recovery sequence completed. Reorienting.")
            self.mode = 'rotate'
            self.recovery_attempts = 0

    def log_status(self, message):
        # Always keep logging current position and goal position as well as distance to goal.
        distance = np.sqrt(
            (self.goal["x"] - self.position["x"]) ** 2 +
            (self.goal["y"] - self.position["y"]) ** 2
        )
        self.get_logger().info(f"{message}")
        self.get_logger().info(
            f"Current => x: {self.position['x']:.2f}, y: {self.position['y']:.2f}, yaw: {self.position['yaw']:.2f}"
        )
        self.get_logger().info(
            f"Target => x: {self.goal['x']:.2f}, y: {self.goal['y']:.2f}, yaw: {self.goal['yaw']:.2f}"
        )
        self.get_logger().info(f"Distance to goal: {distance:.2f}")

    def state_machine(self):
        match self.mode:
            case 'rotate':
                self.rotate_to_goal()
            case 'move':
                self.move_to_goal()
            case 'adjust':
                self.adjust_orientation()
            case 'recover':
                self.recover()


def main(args=None):
    rclpy.init(args=args)
    x = float(input('Enter the final pose value of x(in metres): '))
    y = float(input('Enter the final pose value of y(in metres): '))
    yaw = float(input('Enter the final yaw value of robot(in radians) : '))
    print("Staring movement of robot!")

    node = MovementManager(x, y, yaw)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

