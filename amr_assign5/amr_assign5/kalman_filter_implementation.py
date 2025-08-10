import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion
import numpy as np
import math

class KalmanFilter:
    def __init__(self):
        self.x = np.zeros((3, 1))  # [x, y, theta]
        self.P = np.eye(3)

    def predict(self, A, B, u, Q):
        self.x = A @ self.x + B @ u
        self.P = A @ self.P @ A.T + Q

    def update(self, z, H, R):
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(len(self.x)) - K @ H) @ self.P

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.kf = KalmanFilter()
        self.filter_initialized = False

        self.initial_pose = None
        self.last_odom_time = None

        self.tag_positions = {
            "tag_1": (2.0, 3.0),
            "tag_2": (4.0, 1.0),
            "tag_3": (1.0, 5.0)
        }

        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/rfid', self.rfid_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/kalman_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_estimated_pose)

    def initial_pose_callback(self, msg):
        self.initial_pose = msg
        self.get_logger().info("Initial pose received.")
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        self.get_logger().info(f"Initial pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

    def odom_callback(self, msg):
        current_time = self.get_clock().now()

        if not self.filter_initialized:
            # Use odometry pose to initialize Kalman filter
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
            self.kf.x = np.array([[x], [y], [yaw]])
            self.kf.P = np.eye(3) * 0.01
            self.filter_initialized = True
            self.get_logger().info("Kalman filter automatically initialized using odometry.")
            self.get_logger().info(f"Initial state: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
            self.last_odom_time = current_time
            return

        dt = 0.1
        if self.last_odom_time:
            dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = current_time

        vx = msg.twist.twist.linear.x
        vth = msg.twist.twist.angular.z

        theta = self.kf.x[2, 0]
        A = np.eye(3)
        B = np.array([
            [dt * math.cos(theta), 0],
            [dt * math.sin(theta), 0],
            [0, dt]
        ])
        u = np.array([[vx], [vth]])
        Q = np.diag([0.1, 0.1, 0.05])

        self.kf.predict(A, B, u, Q)
        self.get_logger().info(f"Odometry: vx={vx:.2f}, vth={vth:.2f}, dt={dt:.2f}")
        self.get_logger().info(f"State after predict: x={self.kf.x[0,0]:.2f}, y={self.kf.x[1,0]:.2f}, theta={self.kf.x[2,0]:.2f}")

    def rfid_callback(self, msg):
        tag_id = msg.data
        self.get_logger().info(f"RFID tag detected: {tag_id}")
        if tag_id not in self.tag_positions:
            self.get_logger().warn(f"Unknown RFID tag ID: {tag_id}")
            return

        tag_x, tag_y = self.tag_positions[tag_id]
        z = np.array([[tag_x], [tag_y]])
        H = np.array([[1, 0, 0], [0, 1, 0]])
        R = np.diag([0.5, 0.5])

        if not self.filter_initialized:
            self.get_logger().warn("RFID received but filter not initialized. Ignoring update.")
            return

        self.kf.update(z, H, R)
        self.get_logger().info(f"RFID update: tag={tag_id}, tag_pos=({tag_x},{tag_y})")
        self.get_logger().info(f"State after update: x={self.kf.x[0,0]:.2f}, y={self.kf.x[1,0]:.2f}, theta={self.kf.x[2,0]:.2f}")

    def publish_estimated_pose(self):
        if not self.filter_initialized:
            self.get_logger().info("Waiting for Kalman filter initialization before publishing pose.")
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        pose.pose.position.x = float(self.kf.x[0])
        pose.pose.position.y = float(self.kf.x[1])
        pose.pose.position.z = 0.0

        yaw = float(self.kf.x[2])
        q = self.yaw_to_quaternion(yaw)
        pose.pose.orientation = q

        self.pose_pub.publish(pose)
        self.get_logger().info(f"Published pose: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}, yaw={yaw:.2f}")

    def get_yaw_from_quaternion(self, q):
        quaternion = (q.x, q.y, q.z, q.w)
        _, _, yaw = euler_from_quaternion(quaternion)
        return yaw

    def yaw_to_quaternion(self, yaw):
        q = [0.0] * 4
        q[0] = 0.0
        q[1] = 0.0
        q[2] = math.sin(yaw / 2.0)
        q[3] = math.cos(yaw / 2.0)
        from geometry_msgs.msg import Quaternion
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()