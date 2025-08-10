import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import tf_transformations


class NavigationManager(Node):
    def __init__(self):
        super().__init__("navigation_node")

        # parameters
        self.angle_tolerance = 0.01
        self.position_tolerance = 0.5

        
        self.current_state = 'turning'

        # Robots current position
        self.robot_pose = {
            "x": 0.0,"y": 0.0,"yaw": 0.0
            # "x": 2.0,"y": -3.0,"yaw": -2.0
        }

        # goal coordinates and angle
        self.target_pose = {
            "x": 5.0,"y": 5.0,"yaw": -2.0
            # "x": 0.0,"y": 0.0,"yaw": 0.0

        }

        # Subscribers and publishers
        self.odom_listener = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer to execute behavior based on state
        self.create_timer(0.1, self.state_controller)

    def odom_callback(self, msg):
        self.robot_pose["x"] = msg.pose.pose.position.x
        self.robot_pose["y"] = msg.pose.pose.position.y

        quat = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)
        self.robot_pose["yaw"] = yaw

    def face_target(self):
        #Rotating robott to face the target position
        twist_msg = Twist()
        desired_yaw = np.arctan2(
            self.target_pose["y"] - self.robot_pose["y"],
            self.target_pose["x"] - self.robot_pose["x"]
        )

        yaw_error = desired_yaw - self.robot_pose["yaw"]

        if abs(yaw_error) > self.angle_tolerance:
            twist_msg.angular.z = 1.0
            self.cmd_vel_pub.publish(twist_msg)
            self.get_logger().info("Turning to face the goal...")
        else:
            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            self.current_state = 'driving'
            self.get_logger().info("Oriented towards the goal!")

    def drive_to_target(self):
        # Driving robot towards the goal
        twist_msg = Twist()
        dist = np.sqrt(
            (self.target_pose["x"] - self.robot_pose["x"]) ** 2 +
            (self.target_pose["y"] - self.robot_pose["y"]) ** 2
        )

        if dist > self.position_tolerance:
            twist_msg.linear.x = 1.0
            self.cmd_vel_pub.publish(twist_msg)
            self.get_logger().info(f"Approaching target. Distance: {dist:.2f}")

            # Optional transition logic
            if 1.95 < dist < 2.0:
                twist_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(twist_msg)
                self.current_state = 'turning'

        else:
            twist_msg.linear.x = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            self.current_state = 'finishing'

    def align_yaw(self):
        #Aligning the robot with the goal yaw
        
        self.current_state = 'idle'
        self.get_logger().info("Target reached. Idle state entered.")

    def state_controller(self):
        #Switch cases for the robot

        
        match self.current_state:    
            case 'turning':
                self.face_target()
                
         
            case 'driving':
                self.drive_to_target()
                
                
            case 'finishing':
                self.align_yaw()
                


def main(args=None):
    rclpy.init(args=args)
    controller_node = NavigationManager()
    rclpy.spin(controller_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
