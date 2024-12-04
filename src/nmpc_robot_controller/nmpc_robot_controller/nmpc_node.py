#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Point as GeoPoint  # Avoid conflicts with sympy
from sensor_msgs.msg import LaserScan
import numpy as np
from casadi import *
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import time
import csv  # Import CSV module
import tf2_ros
from tf_transformations import euler_from_quaternion
import math
from sklearn.linear_model import LinearRegression
from scipy.linalg import block_diag

from obstacle_detector.msg import Obstacles

from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

from .functions.path_planner import *

import torch
import torch.nn as nn
from .functions.policy_model import PolicyModel  # Import your PyTorch policy model class

import subprocess
import re

from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import ComputePathToPose

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2


class NMPCController(Node):

    def __init__(self):
        super().__init__('nmpc_controller')

        # NMPC Parameters
        self.Ts = 0.3  # Sampling time
        self.N = 60  # Prediction horizon
        self.nx = 4  # State dimension (x, y, theta,s)
        self.nu = 3  # Input dimension (v, omega, w)

        self.dt = 0
        self.start = 0
        self.end = 0

        # Bounds
        self.umax = np.array([1, 1])    # Upper bounds on controls
        self.lb_u = np.array([0, -1])   # Lower bounds on controls
        self.ub_u = np.array([1, 1])    # Upper bounds on controls
        self.lb_w = 0                   # Lower bound for w
        self.ub_w = 1                   # Upper bound for w
        self.lb_s = 0                   # Lower bound for s (reference trajectory variable)
        self.ub_s = None

        # Weight matrices for cost function
        self.Q = np.diag([1000, 1000, 0])   # State weight
        self.R = np.diag([0.01, 0.01])        # Control input weight
        self.T = 10

        # Other initializations
        # self.cmd_vel_pub = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.ref_path_pub = self.create_publisher(Path, '/ref_path', 10)
        self.ol_path_pub = self.create_publisher(Path, '/ol_path', 10)
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.amcl_pose_callback,10)

        self.global_path_sub = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        # self.obs_sub = self.create_subscription(Obstacles, '/obstacles', self.obs_callback, 10)
        self.obs_inflation = 0.22
        self.obs_list = [
            {"s": 15, "d": 0.2, "r": 0.15 + self.obs_inflation},
            {"s": 23, "d": 0.1, "r": 0.17 + self.obs_inflation},
            {"s": 30, "d": -0.2, "r": 0.2 + self.obs_inflation}
        ]

        # The positions (x, y) will be computed later when the goal is set
        for obs in self.obs_list:
            obs["x"], obs["y"] = None, None  # Placeholder for positions

        self.current_state = np.array([])
        self.goal = np.array([])

        self.initialized = False
        self.u0 = np.array([1, 0])
        self.w0 = 1
        self.s0 = 0

        # Array to store x, y, theta, s values
        self.data_log = []

        self.segment_length = 20
        self.epsilon = 0.15
        self.v_max = 0.1

        self.new_goal_received = False  # Flag to track new goal pose

        # Control loop initialization
        self.last_time = None
        self.control_loop_timer = self.create_timer(0.05, self.control_loop)

        # Create a publisher for the marker
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        # Initialize a unique ID for the marker
        self.marker_id = 0

        self.last_transform_update_time = None

        self.ref_path = None
        self.global_path = None

        self.old_vel = None

        self.obs_position = None  # Replace with your desired position
        self.obs_s = 20
        self.obs_d = 0.1
        self.obs_r = 0.12  # Set the radius

        # Load the CSV data
        self.csv_path = '/home/bertrandt/b.ob/src/nmpc_robot_controller/nmpc_robot_controller/functions/path_data_log_left.csv'  # Replace with the path to your CSV file
        self.path_points = self.load_csv_data(self.csv_path)

        self.obs_position = None  # Replace with your desired position
        self.obs_s = 35
        self.obs_d = 0.58
        self.obs_r = 0.0  # Set the radius

        self.data_log = []  # Stores all the logged data
        self.log_file_path = 'src/closed_loop_mpc_replan.csv'  # Update this path
        
        self.path_received = False  # Flag to indicate if a new path has been received
        self.new_path_points = []  # Store the new path points from /plan

        # Initialize parameters
        self.safety_margin = 0.3  # Safety margin for obstacle avoidance
        self.replan_threshold = 5.5  # Threshold for checking the reference trajectory point

        self.reference_traj = None  # Reference trajectory function
        self.replan_required = False  # Flag to indicate when replanning is needed


        self.pointcloud_pub = self.create_publisher(PointCloud2, '/obstacle_pointcloud', 10)

    def find_closest_obstacle(self):
        """Find the closest obstacle to the current robot position."""
        current_x, current_y = self.current_state[0], self.current_state[1]
        closest_obs = min(
            self.obs_list,
            key=lambda obs: np.sqrt((current_x - obs["x"])**2 + (current_y - obs["y"])**2)
        )
        return np.array([closest_obs["x"], closest_obs["y"], closest_obs["r"]])
    
    def publish_obstacle(self):
        """
        Publish obstacles as PointCloud2 data for the obstacle layer.
        """
        if self.obs_position is None or len(self.obs_position) < 1:
            self.get_logger().error("Obstacle position is not defined correctly.")
            return

        # Create a PointCloud2 message
        pointcloud = PointCloud2()
        pointcloud.header.stamp = self.get_clock().now().to_msg()
        pointcloud.header.frame_id = "map"

        # Extract the obstacle position and radius
        obs_x, obs_y, obs_r = self.obs_position

        obs_r -= 0.06

        # Generate points to approximate the circular obstacle
        points = []
        num_points = 36  # Number of points to approximate the circle
        for angle in np.linspace(0, 2 * np.pi, num_points):
            x = obs_x + obs_r * np.cos(angle)
            y = obs_y + obs_r * np.sin(angle)
            z = 0.0  # Assume flat terrain
            points.append((x, y, z))

        # Convert points to PointCloud2 format
        pointcloud = pc2.create_cloud_xyz32(pointcloud.header, points)

        # Publish the PointCloud2 message
        self.pointcloud_pub.publish(pointcloud)
        
    def publish_circle_marker(self, height=0.1, frame_id='map'):
        for obs in self.obs_list:
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'obstacle_marker'
            marker.id = self.marker_id
            self.marker_id += 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = obs["x"]
            marker.pose.position.y = obs["y"]
            marker.pose.position.z = 0.0  # Ground level
            marker.pose.orientation.w = 1.0

            marker.scale.x = 2 * (obs["r"]-self.obs_inflation)  # Diameter
            marker.scale.y = 2 * (obs["r"]-self.obs_inflation)   # Diameter
            marker.scale.z = height  # Thin cylinder

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            self.marker_publisher.publish(marker)

    def get_current_state(self):
        # Start the ros2 topic echo process
        process = subprocess.Popen(['ros2', 'topic', 'echo', '/amcl_pose'], stdout=subprocess.PIPE, text=True)

        try:
            # Collect lines until a complete message is received
            message_lines = []
            for line in process.stdout:
                message_lines.append(line.strip())
                
                # Check if the end of a message is reached (in ROS, messages are separated by "---")
                if line.strip() == "---":
                    break

            # Combine and print the captured message
            full_message = "\n".join(message_lines)

            # Extract position and orientation using regex
            position_match = re.search(r'position:\s*x: ([\d\-.]+)\s*y: ([\d\-.]+)', full_message)
            orientation_match = re.search(r'orientation:\s*x: ([\d\-.]+)\s*y: ([\d\-.]+)\s*z: ([\d\-.]+)\s*w: ([\d\-.]+)', full_message)

            if position_match and orientation_match:
                x = float(position_match.group(1))
                y = float(position_match.group(2))
                quat_x = float(orientation_match.group(1))
                quat_y = float(orientation_match.group(2))
                quat_z = float(orientation_match.group(3))
                quat_w = float(orientation_match.group(4))

                # Convert quaternion to Euler angles to get theta
                _, _, theta = euler_from_quaternion([quat_x, quat_y, quat_z, quat_w])

                self.current_state = np.array([x, y, theta])

                # Print extracted values
                print("initial position obtained")
                
            # Terminate the process after capturing one message
            process.terminate()
            process.wait(timeout=5)

        except subprocess.TimeoutExpired:
            print("Process did not terminate within the timeout.")
            process.kill()

        finally:
            process.stdout.close()
    
    def load_csv_data(self, file_path):
        # Load CSV using pandas
        df = pd.read_csv(file_path)
        # Convert DataFrame to a list of tuples [(x1, y1), (x2, y2), ...]
        return df[['x', 'y']].values.tolist()

    def amcl_pose_callback(self, msg):
        # Extract position and orientation from the AMCL pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation

        # Convert the quaternion to a list or tuple
        quaternion = [quat.x, quat.y, quat.z, quat.w]

        # Convert quaternion to Euler angles
        _, _, theta = euler_from_quaternion(quaternion)

        # Update the current state
        self.current_state = np.array([x, y, theta])


    def plan_callback(self, msg: Path):
        """
        Capture the path from /plan. Process the first path immediately
        and save subsequent paths for appending during replanning.
        """
        new_points = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            quaternion = [
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ]
            _, _, theta = euler_from_quaternion(quaternion)
            new_points.append((x, y, theta))
        

        if not self.path_points:
            # Process the first path immediately
            self.path_points = new_points
            self.process_path()
            x, y = get_deviated_point(self.global_path, self.obs_s, self.obs_d)
            self.obs_list = np.array([[x,y,self.obs_r]])
            self.obs_position = np.array([x,y,self.obs_r])
        else:
            # Save subsequent paths for appending during replanning
            self.new_path_points = new_points
            self.path_received = True

    def process_path(self):
        """
        Refit the trajectory with the current path points and update the reference trajectory.
        """
        self.global_path, points = LSPB_fit(
            np.array(self.path_points), self.segment_length, self.epsilon, self.v_max
        )

        # Update the reference trajectory
        s = ca.MX.sym('s')
        self.reference_traj = ca.Function('f_s', [s], [f(self.global_path, s)])

        self.get_logger().info("Path and reference trajectory updated.")

        # fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

        # ax1.plot(points[:,0],points[:,1],'ko')

        # for i, segment in enumerate(self.global_path):

        #     if segment.segment_type == 'line':
        #         s = np.linspace(segment.start_time, segment.end_time-0.01, 1000)
        #         x_vals = []
        #         y_vals = []

        #         for s_value in s:
        #             result = self.reference_traj(s_value)
        #             x_vals.append(float(result[0]))
        #             y_vals.append(float(result[1]))

        #         ax1.plot(x_vals, y_vals, 'r-', label="Full Path using f_s")
        #         ax1.set_title(f"Original Segment and Input Positions")
        #         ax1.set_xlabel('x')
        #         ax1.set_ylabel('y')
        #         ax1.grid(True)

        #         tfx = []
        #         tfy = []

        #         for i in range(len(x_vals)):
        #             point = segment.inv_transform_p((x_vals[i],y_vals[i]))
        #             tfx.append(point[0])
        #             tfy.append(point[1])

        #         ax2.plot(tfx,tfy,'g-')


        #     if segment.segment_type == 'parabola':
                
        #         s = np.linspace(segment.start_time, segment.end_time-0.01, 1000)
        #         x_vals = []
        #         y_vals = []

        #         for s_value in s:
        #             result = self.reference_traj(s_value)
        #             x_vals.append(float(result[0]))
        #             y_vals.append(float(result[1]))

        #         ax1.plot(x_vals, y_vals, 'b-', label="Full Path using f_s")
        #         ax1.set_title(f"Original Segment and Input Positions")
        #         ax1.set_xlabel('x')
        #         ax1.set_ylabel('y')
        #         ax1.grid(True)

        #         tfx = []
        #         tfy = []

        #         for i in range(len(x_vals)):
        #             point = segment.inv_transform_p((x_vals[i],y_vals[i]))
        #             tfx.append(point[0])
        #             tfy.append(point[1])

        #         ax2.plot(tfx,tfy,'g-')

        # plt.show()

    def replan_path(self, start_index):
        """
        Replan the path by appending new path points after the specified start_index.
        """
        if not self.path_received:
            self.get_logger().error("No new path received from /plan.")
            return

        # Append the new path points after the start_index
        self.path_points = self.path_points[:start_index] + self.new_path_points

        # Refit the trajectory and update the reference trajectory
        self.process_path()

        # Reset the new path flag
        self.path_received = False

    def check_update_path(self):
        """
        Check if the reference trajectory needs updating based on obstacle proximity.
        """
        ref_point = self.reference_traj(self.s0 + self.replan_threshold)

        # Check for proximity to obstacles
        for obs in self.obs_list:
            obs_x, obs_y, obs_radius = obs
            distance = np.sqrt((ref_point[0] - obs_x) ** 2 + (ref_point[1] - obs_y) ** 2)

            if distance < obs_radius + self.safety_margin and obs_radius > 0:
                self.get_logger().info(
                    f"Obstacle detected at ({obs_x}, {obs_y}). Replanning triggered."
                )
                self.stop_robot()

                # Find the closest point in the path to the reference point
                start_index = self.find_closest_path_point(self.current_state)

                # Trigger replanning
                self.replan_path(start_index)
                break

    def find_closest_path_point(self, ref_point):
        """
        Find the index of the closest path point to the reference point.
        """
        distances = [
            np.sqrt((point[0] - ref_point[0]) ** 2 + (point[1] - ref_point[1]) ** 2)
            for point in self.path_points
        ]
        return np.argmin(distances)

    def goal_pose_callback(self, msg):
        # Use the CSV data instead of generating new path points
        path_points = self.path_points  # This uses the loaded CSV data directly

        # Fit the path using LSPB
        self.global_path, points = LSPB_fit(np.array(path_points), self.segment_length, self.epsilon, self.v_max)
        
        # Create the reference trajectory function
        s = ca.MX.sym('s')
        self.reference_traj = ca.Function('f_s', [s], [f(self.global_path, s)])

        self.ub_s = self.global_path[-1].end_time
        self.new_goal_received = False

        # Set self.goal to the final endpoint of the global path with theta = 0
        final_position = self.global_path[-1].end_point
        self.goal = np.array([final_position[0], final_position[1], 0.0])


    def goal_pose_callback(self, msg):
        self.new_goal_received = True

        x = msg.pose.position.x
        y = msg.pose.position.y
        quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        _, _, theta = euler_from_quaternion(quaternion)
        
        self.goal = np.array([x, y, theta])

    def gamma(self,eta):
        # Define a small threshold to prevent division by zero
        eta_safe = ca.fmax(eta, 1e-10)  # Prevents eta from being exactly zero

        # Calculate non-zero eta case safely
        sqrt_term = ca.sqrt(1 + (2 * eta_safe) ** 2)
        arcsinh_term = ca.arcsinh(2 * eta_safe) / (2 * eta_safe)
        
        # Combine with if_else, guaranteeing no division by zero
        result = ca.if_else(eta != 1e-10, 0.5 * (sqrt_term + arcsinh_term), 1)
        return result

    def g(self,v_max,eta):
        return v_max/self.gamma(eta)


    def control_loop(self):
        now = self.get_clock().now()

        if self.start == 0:
            self.start = time.perf_counter()
        else:
            self.end = time.perf_counter()
            self.dt = self.end - self.start
            self.start = self.end

        if self.current_state.shape[0] <= 0:
            self.get_current_state()
            self.stop_robot()
            return  # Skip this control loop iteration if no state

        if self.goal.shape[0] > 0:
            goal_dist = self.dist(self.current_state, self.goal)
        else:
            goal_dist = 0

        if self.global_path is not None:
            self.publish_reference_path()  # Publish the reference path if available

        if self.global_path is not None and self.dt > 0 and goal_dist > 0.1:
            self.check_update_path()
            self.publish_obstacle()

            # Transform current state into the path-relative coordinates
            x_hat, y_hat, theta_hat, s_hat, eta = T_z(
                self.global_path,
                self.current_state[0],
                self.current_state[1],
                self.current_state[2],
                self.s0,
            )

            # Prepare model input
            if eta >= 0:
                input_data = torch.tensor([[x_hat, y_hat, theta_hat, s_hat, eta]], dtype=torch.float32).to(self.device)
            else:
                input_data = torch.tensor([[x_hat, -y_hat, -theta_hat, s_hat, -eta]], dtype=torch.float32).to(self.device)

            # Perform inference with PyTorch model
            with torch.no_grad():
                usol = self.model(input_data).cpu().numpy()[0]  # Get the first (and only) output

            if eta < 0:
                usol[1] *= -1  # Adjust angular velocity for inverted eta

            # Apply feedback corrections to model predictions
            en, et, phi = error(self.global_path, self.current_state, self.s0)
            Pt = 1.0  # Tangential feedback gain
            Pn = 1.0  # Normal feedback gain

            usol[0] -= Pt * et  # Correct linear velocity based on tangential error
            usol[1] -= Pn * en  # Correct angular velocity based on normal error

            # Scale and adjust the control input
            # usol = self.convert_u(usol)
            usol = [np.clip(float(usol[0]),0,1),np.clip(float(usol[1]),-0.8,0.8),np.clip(float(usol[2]),0,1)]

            # Publish the corrected control command
            self.publish_control(usol)

            # Publish markers and reference path
            self.publish_circle_marker()
            self.publish_reference_path()

            # Update state variables
            self.w0 = usol[2]  # Update path progress rate
            self.s0 += self.dt * self.w0  # Update path progress
        else:
            self.stop_robot()


    def stop_robot(self):
        # Publish zero velocity to stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        print("stopped")


    def dist(self, current, goal):
        return np.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)

    def publish_ol_path(self, path):
        ol_path = Path()
        ol_path.header.stamp = self.get_clock().now().to_msg()
        ol_path.header.frame_id = "map"  # Adjust frame_id to your setup

        for i in range(path.shape[0]):
            val = path[i, :]
            pose = PoseStamped()
            pose.header.stamp = ol_path.header.stamp
            pose.header.frame_id = ol_path.header.frame_id
            pose.pose.position.x = float(val[0])
            pose.pose.position.y = float(val[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # Assume no orientation or flat trajectory
            ol_path.poses.append(pose)

        self.ol_path_pub.publish(ol_path)

    def publish_reference_path(self):
        ref_path = Path()
        ref_path.header.stamp = self.get_clock().now().to_msg()
        ref_path.header.frame_id = "map"  # Adjust frame_id to your setup

        for s in np.linspace(0, self.global_path[-1].end_time, num=100):
            eta_val = self.reference_traj(s)
            pose = PoseStamped()
            pose.header.stamp = ref_path.header.stamp
            pose.header.frame_id = ref_path.header.frame_id
            pose.pose.position.x = float(eta_val[0])
            pose.pose.position.y = float(eta_val[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # Assume no orientation or flat trajectory
            ref_path.poses.append(pose)

        self.ref_path_pub.publish(ref_path)

    def publish_control(self, control_input):
        twist_msg = Twist()
        twist_msg.linear.x = control_input[0]
        twist_msg.angular.z = control_input[1]
        self.cmd_vel_pub.publish(twist_msg)

    def convert_u(self,usol):
        u = [0,0,0]

        u[0] = 0.02 + np.clip(usol[0],0,1)*(0.15-0.02)
        u[1] = np.sign(usol[1])*0.05 + np.clip(usol[1],-0.8,0.8)*(0.15-0.05)
        u[2] = 0.9 * np.clip(usol[2],0,1)

        return u

    def dist(self,current,goal):
        return np.sqrt((current[0]-goal[0])**2 + (current[1]-goal[1])**2)

    def publish_ol_path(self, path):
        ol_path = Path()
        ol_path.header.stamp = self.get_clock().now().to_msg()
        ol_path.header.frame_id = "map"  # Adjust frame_id to your setup

        for i in range(path.shape[0]):
            val = path[i, :]
            pose = PoseStamped()
            pose.header.stamp = ol_path.header.stamp
            pose.header.frame_id = ol_path.header.frame_id
            pose.pose.position.x = float(val[0])
            pose.pose.position.y = float(val[1])
            pose.pose.position.z = 0.0
            # Assume no orientation or flat trajectory, set quaternion accordingly
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            ol_path.poses.append(pose)

        self.ol_path_pub.publish(ol_path)

    def publish_control(self, control_input):
        twist_msg = Twist()
        twist_msg.linear.x = control_input[0]
        twist_msg.angular.z = control_input[1]
        self.cmd_vel_pub.publish(twist_msg)

    def save_to_csv(self, filename='path_data_log.csv'):
        """Save the logged x, y, theta, s data to a CSV file."""
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y'])  # Header
            writer.writerows(self.data_log)
        self.get_logger().info(f'Data saved to {filename}')



def main(args=None):
    rclpy.init(args=args)
    nmpc_controller = NMPCController()

    rclpy.spin(nmpc_controller)

    nmpc_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
