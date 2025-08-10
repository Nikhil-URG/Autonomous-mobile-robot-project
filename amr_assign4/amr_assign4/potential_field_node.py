import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
import numpy as np
import math

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point
from tf_transformations import euler_from_quaternion

class PotentialFieldPlanner(Node):
    def __init__(self):
        super().__init__('potential_field_planner')

        # Tried using the system time here but due to inconsistencies between
        # sim time and clock time data could not be synced for transformations
        # So using sim_time here for consistency
        self.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        
        # Increasing kr and rho0 will basically cause robot to detect obstacle early
        # and also increases the repulsive force
       
        self.ka = 0.5  # Attractive gain
        self.kr = 1.5  # Repulsive gain (Increased from 1.2) - Stronger push away from obstacles
        self.rho0 = 1.0 # Obstacle influence radius (meters) (Increased from 0.7) - Start reacting earlier
        self.goal_tolerance = 0.1 # Distance to goal to consider reached (meters)
        self.max_linear_vel = 0.3 # m/s
        self.max_angular_vel = 0.5 # rad/s
        self.orientation_kp = 1.0 # Proportional gain for final orientation
        self.orientation_threshold = 0.05 # radians, threshold to consider final orientation reached

        #Defining the goal pose and position here in the odom frame
        self.global_goal_x = 4.0 # in metres
        self.global_goal_y = 10.0 # in metres
        self.global_goal_yaw = -1.0 # in radians

        self.get_logger().info(f"Fixed Goal Set: x={self.global_goal_x}, y={self.global_goal_y}, yaw={self.global_goal_yaw} rad")

        # Store the goal as a PoseStamped for easier TF transformations
        self.current_goal_pose_stamped = PoseStamped()
        self.current_goal_pose_stamped.header.frame_id = 'odom' # The frame of the goal
        self.current_goal_pose_stamped.pose.position.x = self.global_goal_x
        self.current_goal_pose_stamped.pose.position.y = self.global_goal_y
        # No need to set orientation for attractive force, but good to store 
        # so that it can be used to perform the final orientation

        # Publishers and Subscribers for scan data and publishing velocity
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Calling function every 0.1second for
        # dynamical, continuous motion and updates to velocity based on obstacles detected
        self.timer = self.create_timer(0.1, self.publish_cmd) # Call publish_cmd every 0.1 seconds

        # TF Buffer and Listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State Variables for storing current robot position
        self.scan_data = None # Store the latest scan message
        self.robot_x = 0.0 # Current robot X position in odom frame
        self.robot_y = 0.0 # Current robot Y position in odom frame
        self.robot_yaw = 0.0 # Current robot Yaw orientation in odom frame
        self.goal_position_reached = False # Flag to check if destination is reached

        self.get_logger().info("Potential Field Planner Node has been started successfully!.")

    def scan_callback(self, msg: LaserScan):
        # Just get the data from topic and store in scan_data
        self.scan_data = msg

    def get_current_pose(self):
        # get the current robot configutation
        try:
            # Look up the transform from 'odom' to 'base_link'
            # Requesting the LATEST available transform (rclpy.time.Time() means Time(0))
            transform = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                rclpy.time.Time(), # Use Time(0) for latest available transform
                timeout=rclpy.duration.Duration(seconds=0.1) # Add a small timeout
            )
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            
            # Convert quaternion to yaw for current robot orientation
            orientation_q = transform.transform.rotation
            # euler_from_quaternion returns (roll, pitch, yaw)
            _, _, self.robot_yaw = euler_from_quaternion([
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            ])
            return True
        except TransformException as ex:
            # Log warnings if transform is not available, but throttle to avoid spam
            self.get_logger().warn(f"Could not transform 'odom' to 'base_link': {ex}", throttle_duration_sec=1.0)
            return False


    def publish_cmd(self):
        # Here we can implement the repusive and attractive forces logic for moving the robot
        # Ensure we have scan data and robot pose before proceeding
        if self.scan_data is None:
            self.get_logger().info("No scan data received yet. Waiting...", throttle_duration_sec=1.0)
            self.pub.publish(Twist()) # Stop the robot
            return

        if not self.get_current_pose():
            self.get_logger().info("Robot pose not available. Waiting...", throttle_duration_sec=1.1)
            self.pub.publish(Twist()) # Stop the robot
            return


        # Check if the goal position (x,y) has been reached
        global_distance_to_goal = math.sqrt(
            (self.global_goal_x - self.robot_x)**2 + 
            (self.global_goal_y - self.robot_y)**2
        )

        if global_distance_to_goal < self.goal_tolerance:
            if not self.goal_position_reached:
                self.get_logger().info("Goal position reached! Now adjusting orientation.")
                self.goal_position_reached = True
            self.adjust_final_orientation() # Handle final orientation
            return # Stop further potential field calculations

        # If goal position is not yet reached, proceed with potential field calculations
        self.goal_position_reached = False # Reset flag if robot moves away from goal area

        # Transform the global goal from 'odom' frame to 'base_link' frame
        # This is necessary because potential field calculations are done relative to the robot's current frame (base_link)
        try:
            # Set the timestamp of the goal pose to Time(0) to request the latest transform
            self.current_goal_pose_stamped.header.stamp = rclpy.time.Time().to_msg()
            goal_in_base_link_stamped = self.tf_buffer.transform(
                self.current_goal_pose_stamped,
                'base_link',
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            goal_in_base_link_x = goal_in_base_link_stamped.pose.position.x
            goal_in_base_link_y = goal_in_base_link_stamped.pose.position.y
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform goal to base_link: {ex}", throttle_duration_sec=1.0)
            self.pub.publish(Twist()) # Stop the robot
            return

        # Calculate Attractive Force (in base_link frame)
        # Robot is at (0,0) in its own base_link frame
        q_robot_bl = np.array([0.0, 0.0])
        q_goal_bl = np.array([goal_in_base_link_x, goal_in_base_link_y])
        
        diff_attr = q_goal_bl - q_robot_bl
        dist_attr = np.linalg.norm(diff_attr)

        force_attr = np.array([0.0, 0.0])
        if dist_attr != 0:
            # The formula is -ka * (q - q_goal) / ||q - q_goal||
            # Which is equivalent to ka * (q_goal - q) / ||q_goal - q||
            force_attr = self.ka * (diff_attr / dist_attr)
        # else: if dist_attr is 0, force_attr remains [0,0]

        # Calculate Repulsive Force (in base_link frame)
        force_rep = np.array([0.0, 0.0])

        # Look up transform from laser frame to base_link *once* outside the loop
        try:
            # Requesting the LATEST available transform (rclpy.time.Time() means Time(0))
            transform_laser_to_base_link = self.tf_buffer.lookup_transform(
                'base_link',
                self.scan_data.header.frame_id, # e.g., 'laser_link'
                rclpy.time.Time(), # Use Time(0) for latest available transform
                timeout=rclpy.duration.Duration(seconds=0.1) # Add a small timeout
            )
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform {self.scan_data.header.frame_id} to base_link for scan: {ex}", throttle_duration_sec=1.0)
            self.pub.publish(Twist()) # Stop the robot
            return
        # Look at the scan data and try to extract the ranges which are looking at an obstacle
        ranges = np.array(self.scan_data.ranges)
        angles = np.linspace(self.scan_data.angle_min, self.scan_data.angle_max, len(ranges))

        for r, theta in zip(ranges, angles):
            # Ignore data that is not useful for us
            if not np.isfinite(r) or r == 0.0 or r > self.rho0:
                continue

            point_laser = PointStamped()
            point_laser.header = self.scan_data.header # Use scan's header for stamp and frame_id
            # Set the timestamp of the laser point to Time(0) to request the latest transform
            point_laser.header.stamp = rclpy.time.Time().to_msg()
            point_laser.point.x = r * np.cos(theta)
            point_laser.point.y = r * np.sin(theta)
            point_laser.point.z = 0.0 # Since laster is working in a 2D plane assigning z to 0

            # Transform the obstacle point from laser_link to base_link
            obstacle_data_from_base = do_transform_point(
                point_laser,
                transform_laser_to_base_link
            )
            q_obstacle_bl = np.array([obstacle_data_from_base.point.x, obstacle_data_from_base.point.y])


            diff_rep = q_robot_bl - q_obstacle_bl # Vector from obstacle to robot
            dist_rep = np.linalg.norm(diff_rep)

            if dist_rep == 0: # Avoid division by zero if on top of obstacle
                self.get_logger().warn("Robot on top of obstacle, stopping!")
                self.pub.publish(Twist())
                return

            term1 = (1.0 / dist_rep) - (1.0 / self.rho0)
            term2 = 1.0 / (dist_rep**2)
            
            rep_dir = diff_rep / dist_rep # Unit vector from obstacle to robot (pushing away)

            repulsive_component_x = self.kr * term1 * term2 * rep_dir[0]
            repulsive_component_y = self.kr * term1 * term2 * rep_dir[1]
            
            force_rep[0] += repulsive_component_x
            force_rep[1] += repulsive_component_y

        # Total Force (interpreted as desired velocity vector)
        total_force = force_attr + force_rep

        # Convert total force vector to linear and angular velocities for a differential drive robot
        linear_x = np.linalg.norm(total_force)
        angular_z = np.arctan2(total_force[1], total_force[0]) # Angle of the desired velocity vector

        # Apply velocity limits
        linear_x = np.clip(linear_x, 0.0, self.max_linear_vel) # Linear velocity should generally be positive
        angular_z = np.clip(angular_z, -self.max_angular_vel, self.max_angular_vel)

        # Publish Twist message
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = float(linear_x)
        cmd_vel_msg.angular.z = float(angular_z)
        self.pub.publish(cmd_vel_msg)

        # For debugging purpose to check if forces are not allowing robot to move if it gets into deadlock
        # self.get_logger().info(f"Robot Pos (odom): ({self.robot_x:.2f}, {self.robot_y:.2f}, {self.robot_yaw:.2f})")
        # self.get_logger().info(f"Goal (BL): ({goal_in_base_link_x:.2f}, {goal_in_base_link_y:.2f})")
        self.get_logger().info(f"Attr Force: ({force_attr[0]:.2f}, {force_attr[1]:.2f})")
        self.get_logger().info(f"Rep Force: ({force_rep[0]:.2f}, {force_rep[1]:.2f})")
        self.get_logger().info(f"Total Force: ({total_force[0]:.2f}, {total_force[1]:.2f})")
        self.get_logger().info(f"Setting final values as - linear_x={linear_x:.2f}, angular_z={angular_z:.2f})")


    def adjust_final_orientation(self):
        # Here we have reached the goal position and basically trying to align with goal orientation
        # Normalize angles to -pi to pi range
        robot_yaw_norm = math.atan2(math.sin(self.robot_yaw), math.cos(self.robot_yaw))
        goal_yaw_norm = math.atan2(math.sin(self.global_goal_yaw), math.cos(self.global_goal_yaw))
        
        # Calculate shortest angular distance
        angular_error = goal_yaw_norm - robot_yaw_norm
        # Normalize angular error to be within [-pi, pi]
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))

        cmd_vel_msg = Twist()

        if abs(angular_error) > self.orientation_threshold:
            # Proportional control for angular velocity
            angular_z = self.orientation_kp * angular_error
            angular_z = np.clip(angular_z, -self.max_angular_vel, self.max_angular_vel)
            cmd_vel_msg.angular.z = float(angular_z)
            self.get_logger().info(f"Adjusting orientation. Current yaw: {robot_yaw_norm:.2f}, Moving to yaw: {goal_yaw_norm:.2f}, Error: {angular_error:.2f}, Final command sent to robot: {angular_z:.2f}", throttle_duration_sec=0.5)
        else:
            self.get_logger().info("Final orientation reached. Stopping.", throttle_duration_sec=1.0)
            # Robot is at goal position and correct orientation, so publish zero velocity
            # cmd_vel_msg remains zero, effectively stopping the robot.

        self.pub.publish(cmd_vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldPlanner()
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node() 
        rclpy.shutdown()

if __name__ == '__main__':
    main()
