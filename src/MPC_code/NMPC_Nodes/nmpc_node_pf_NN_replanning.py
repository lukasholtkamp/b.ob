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

# from obstacle_detector.msg import Obstacles

from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

from .functions.path_planner import *

import torch
import torch.nn as nn
<<<<<<< HEAD
from .functions.policy_model import PolicyModel  # Import your PyTorch policy model class
=======
from .functions.policy_model import (
    PolicyModel,
)  # Import your PyTorch policy model class
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

import subprocess
import re

from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import ComputePathToPose

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2


class NMPCController(Node):

    def __init__(self):
        super().__init__("nmpc_controller")

        # NMPC Parameters
        self.Ts = 0.3  # Sampling time
        self.N = 20  # Prediction horizon
        self.nx = 4  # State dimension (x, y, theta,s)
        self.nu = 3  # Input dimension (v, omega, w)

        self.dt = 0
        self.start = 0
        self.end = 0

        # Bounds
        self.umax = np.array([1, 1])  # Upper bounds on controls
        self.lb_u = np.array([0, -1])  # Lower bounds on controls
        self.ub_u = np.array([1, 1])  # Upper bounds on controls
        self.lb_w = 0  # Lower bound for w
        self.ub_w = 1  # Upper bound for w
        self.lb_s = 0  # Lower bound for s (reference trajectory variable)
        self.ub_s = None

        # Weight matrices for cost function
        self.Q = np.diag([1000, 1000, 0])  # State weight
        self.R = np.diag([0.01, 0.01])  # Control input weight
        self.T = 10

        # Other initializations
        # self.cmd_vel_pub = self.create_publisher(
        #     Twist, "/diffbot_base_controller/cmd_vel_unstamped", 10
        # )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.ref_path_pub = self.create_publisher(Path, "/ref_path", 10)
        self.ol_path_pub = self.create_publisher(Path, "/ol_path", 10)
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.amcl_pose_callback, 10
        )

        self.global_path_sub = self.create_subscription(
            Path, "/plan", self.plan_callback, 10
        )
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_pose_callback, 10
        )

        # self.obs_sub = self.create_subscription(Obstacles, '/obstacles', self.obs_callback, 10)
        self.max_obs = 1
        self.obs_list = np.zeros((self.max_obs, 3))

        self.current_state = np.array([])
        self.goal = np.array([])

        self.initialized = False
        self.u0 = np.array([1, 0])
        self.w0 = 1
        self.s0 = 0
        self.s_real = 0

        self.segment_length = 20
        self.epsilon = 0.15
        self.v_max = 0.1

        self.new_goal_received = False  # Flag to track new goal pose

        # Control loop initialization
        self.last_time = None
        self.control_loop_timer = self.create_timer(0.05, self.control_loop)

        # Create a publisher for the marker
        self.marker_publisher = self.create_publisher(
            Marker, "/visualization_marker", 10
        )
        # Initialize a unique ID for the marker
        self.marker_id = 0

        self.last_transform_update_time = None

        self.ref_path = None
        self.global_path = None

        self.obs_position = None  # Replace with your desired position
        self.obs_s = 20
        self.obs_d = 0.1
        self.obs_r = 0.12  # Set the radius
<<<<<<< HEAD
=======

        self.obs_inflation = 0.3
        self.obs_list = [
            {"s": 15, "d": 0.23, "r": 0.15},
            {"s": 30, "d": -0.3, "r": 0.2},
        ]

        # Load PyTorch model
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu"
        )  # Use GPU if available
        model_path = "/home/bertrandt/b.ob/src/MPC_code/IL/DPL/models/final_policy.pth"  # Replace with the actual path to your final policy
        self.model = PolicyModel(
            input_dim=5, output_dim=3
        )  # Adjust dimensions as per your model
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()  # Set to evaluation mode
        self.model.to(self.device)  # Move model to appropriate device
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

        # Load PyTorch model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # Use GPU if available
        model_path = "/home/bertrandt/b.ob/src/MPC_code/IL/DPL/models/final_policy.pth"  # Replace with the actual path to your final policy
        self.model = PolicyModel(input_dim=5, output_dim=3)  # Adjust dimensions as per your model
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()  # Set to evaluation mode
        self.model.to(self.device)  # Move model to appropriate device
        
        self.path_received = False  # Flag to indicate if a new path has been received
        self.new_path_points = []  # Store the new path points from /plan

        # Initialize parameters
        self.safety_margin = 0.3  # Safety margin for obstacle avoidance
<<<<<<< HEAD
        self.replan_threshold = 5.5  # Threshold for checking the reference trajectory point
=======
        self.replan_threshold = (
            5.5  # Threshold for checking the reference trajectory point
        )
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097
        self.path_points = []  # Path points list
        self.reference_traj = None  # Reference trajectory function
        self.replan_required = False  # Flag to indicate when replanning is needed

        self.pointcloud_pub = self.create_publisher(
            PointCloud2, "/obstacle_pointcloud", 10
        )

        self.data_log = []  # Stores all the logged data
        self.log_file_path = "src/closed_loop_nn_pf_replan.csv"  # Update this path

        self.path_log = []
        self.pc_publish_timer = self.create_timer(0.01, self.publish_obstacle)

    def publish_obstacle(self):
        if not self.obs_list:
            return
        if "x" not in self.obs_list[0]:
            return

        # Assuming at least two obstacles in self.obs_list
        # This logic:
        # - If s_real < obs_list[0]['s']: Publish only the first obstacle
        # - If s_real >= obs_list[0]['s']: Publish only the second obstacle
        # You can extend this logic for more obstacles if needed.

        pointcloud = PointCloud2()
        pointcloud.header.stamp = self.get_clock().now().to_msg()
        pointcloud.header.frame_id = "map"

<<<<<<< HEAD
        # Extract the obstacle position and radius
        obs_x, obs_y, obs_r = self.obs_position

        obs_r -= 0.06

        # Generate points to approximate the circular obstacle
=======
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097
        points = []
        num_points = 36  # Points per obstacle circle

        if self.s_real < self.obs_list[0]["s"]:
            # Publish the first obstacle only
            obs = self.obs_list[0]
            obs_x = obs["x"]
            obs_y = obs["y"]
            obs_r = obs["r"] - 0.06
            for angle in np.linspace(0, 2 * np.pi, num_points, endpoint=False):
                x = obs_x + obs_r * np.cos(angle)
                y = obs_y + obs_r * np.sin(angle)
                z = 0.0
                points.append((x, y, z))
        else:
            # Once we've passed the s of the first obstacle, publish the second obstacle
            # Adjust index or logic as needed if you have more obstacles
            if len(self.obs_list) > 1:
                obs = self.obs_list[1]
                obs_x = obs["x"]
                obs_y = obs["y"]
                obs_r = obs["r"] - 0.06
                for angle in np.linspace(0, 2 * np.pi, num_points, endpoint=False):
                    x = obs_x + obs_r * np.cos(angle)
                    y = obs_y + obs_r * np.sin(angle)
                    z = 0.0
                    points.append((x, y, z))

        if points:
            pointcloud = pc2.create_cloud_xyz32(pointcloud.header, points)
            self.pointcloud_pub.publish(pointcloud)



    def publish_circle_marker(self):
        if not self.obs_list:
            return

        marker_id = 0
        for obs in self.obs_list:
            obs_x = obs["x"]
            obs_y = obs["y"]
            obs_r = obs["r"]

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacle_marker"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = obs_x
            marker.pose.position.y = obs_y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 2 * obs_r
            marker.scale.y = 2 * obs_r
            marker.scale.z = 0.1

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            self.marker_publisher.publish(marker)


    def get_current_state(self):
        # Start the ros2 topic echo process
        process = subprocess.Popen(
            ["ros2", "topic", "echo", "/amcl_pose"], stdout=subprocess.PIPE, text=True
        )

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
            position_match = re.search(
                r"position:\s*x: ([\d\-.]+)\s*y: ([\d\-.]+)", full_message
            )
            orientation_match = re.search(
                r"orientation:\s*x: ([\d\-.]+)\s*y: ([\d\-.]+)\s*z: ([\d\-.]+)\s*w: ([\d\-.]+)",
                full_message,
            )

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

    def obs_callback(self, msg):

        # Initialize an array to store obstacle information
        obs = np.zeros((len(msg.circles), 3))

        # Extract the x, y, radius for each circle and store it in the obs array
        for i in range(len(msg.circles)):
            obs[i] = [
                msg.circles[i].center.x,
                msg.circles[i].center.y,
                msg.circles[i].radius,
            ]

        # Calculate the difference between the obstacle positions and the current position
        try:
            diff = obs[:, :2] - np.tile(
                self.current_state[:2], (obs[:, :2].shape[0], 1)
            )

            # Calculate the Euclidean distance
            distances = np.linalg.norm(diff, axis=1)

            # Sort the obs array according to the distances
            sorted_indices = np.argsort(distances)
            obs_sorted = obs[sorted_indices]

            if obs_sorted.shape[0] < self.max_obs:
                obs_sorted = np.vstack(
                    (obs_sorted, np.zeros((self.max_obs - obs_sorted.shape[0], 3)))
                )

            self.obs_list = obs_sorted[: self.max_obs, :]

        except:
            self.obs_list = np.zeros((self.max_obs, 3))

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
            self.path_log = new_points
            self.save_to_csv()
            self.process_path()
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
        s = ca.MX.sym("s")
        self.reference_traj = ca.Function("f_s", [s], [f(self.global_path, s)])
        self.get_logger().info("Path and reference trajectory updated.")

        # Compute obstacle positions only once if they are not already computed
        # This ensures we do not update them as the path changes later.
        # Check if obs_list doesn't have "x" and "y" keys. Compute if needed.
        for obs in self.obs_list:
            if "x" not in obs or "y" not in obs:
                obs_x, obs_y = get_deviated_point(self.global_path, obs["s"], obs["d"])
                obs["x"] = obs_x
                obs["y"] = obs_y
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

        self.path_log = self.path_points
        self.save_to_csv_replan()

        # Reset the new path flag
        self.path_received = False

    def check_update_path(self):
        """
        Check if the reference trajectory needs updating based on obstacle proximity.
        Now handles multiple obstacles from self.obs_list.
        """
        if self.reference_traj is None:
            return

        ref_point = self.reference_traj(self.s0 + self.replan_threshold)

        for obs in self.obs_list:
            obs_x, obs_y = get_deviated_point(self.global_path, obs["s"], obs["d"])
            obs_radius = obs["r"]
            distance = np.sqrt((ref_point[0] - obs_x) ** 2 + (ref_point[1] - obs_y) ** 2)

            if distance < obs_radius + self.safety_margin and obs_radius > 0:
                self.get_logger().info(
                    f"Obstacle detected near s={obs['s']}, d={obs['d']}. Replanning triggered."
                )
                self.stop_robot()
                start_index = self.find_closest_path_point(self.current_state)
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
        self.new_goal_received = True

        x = msg.pose.position.x
        y = msg.pose.position.y
        quaternion = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        _, _, theta = euler_from_quaternion(quaternion)

        self.goal = np.array([x, y, theta])

        print(self.goal)

    def gamma(self, eta):
        # Define a small threshold to prevent division by zero
        eta_safe = ca.fmax(eta, 1e-10)  # Prevents eta from being exactly zero

        # Calculate non-zero eta case safely
        sqrt_term = ca.sqrt(1 + (2 * eta_safe) ** 2)
        arcsinh_term = ca.arcsinh(2 * eta_safe) / (2 * eta_safe)

        # Combine with if_else, guaranteeing no division by zero
        result = ca.if_else(eta != 1e-10, 0.5 * (sqrt_term + arcsinh_term), 1)
        return result

    def g(self, v_max, eta):
        return v_max / self.gamma(eta)

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
<<<<<<< HEAD

        if self.global_path is not None and self.dt > 0 and goal_dist > 0.2:
=======
            self.initialized = True

        if self.global_path is not None and self.dt > 0 and goal_dist > 0.1:
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097
            self.check_update_path()

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
<<<<<<< HEAD
                input_data = torch.tensor([[x_hat, y_hat, theta_hat, s_hat, eta]], dtype=torch.float32).to(self.device)
            else:
                input_data = torch.tensor([[x_hat, -y_hat, -theta_hat, s_hat, -eta]], dtype=torch.float32).to(self.device)

            # Perform inference with PyTorch model
            with torch.no_grad():
                usol = self.model(input_data).cpu().numpy()[0]  # Get the first (and only) output
=======
                input_data = torch.tensor(
                    [[x_hat, y_hat, theta_hat, s_hat, eta]], dtype=torch.float32
                ).to(self.device)
            else:
                input_data = torch.tensor(
                    [[x_hat, -y_hat, -theta_hat, s_hat, -eta]], dtype=torch.float32
                ).to(self.device)

            # Perform inference with PyTorch model
            with torch.no_grad():
                usol = (
                    self.model(input_data).cpu().numpy()[0]
                )  # Get the first (and only) output
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

            if eta < 0:
                usol[1] *= -1  # Adjust angular velocity for inverted eta

<<<<<<< HEAD
            # Apply feedback corrections to model predictions
            en, et, phi = error(self.global_path, self.current_state, self.s0)
            Pt = 1.0  # Tangential feedback gain
            Pn = 1.0  # Normal feedback gain

            usol[0] -= Pt * et  # Correct linear velocity based on tangential error
            usol[1] -= Pn * en  # Correct angular velocity based on normal error

            # Scale and adjust the control input
            # usol = self.convert_u(usol)
            usol = [np.clip(float(usol[0]),0,1),np.clip(float(usol[1]),-0.8,0.8),np.clip(float(usol[2]),0,1)]
=======
            # # Apply feedback corrections to model predictions
            # en, et, phi = error(self.global_path, self.current_state, self.s0)
            # Pt = 1.0  # Tangential feedback gain
            # Pn = 1.0  # Normal feedback gain

            # usol[0] -= Pt * et  # Correct linear velocity based on tangential error
            # usol[1] -= Pn * en  # Correct angular velocity based on normal error

            # Scale and adjust the control input
            # usol = self.convert_u(usol)
            usol = [
                np.clip(float(usol[0]), 0, 1),
                np.clip(float(usol[1]), -0.8, 0.8),
                np.clip(float(usol[2]), 0, 1),
            ]

            usol_real = usol.copy()
            self.s_real += self.dt * usol[2]

            # usol = self.convert_u(usol)
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

            # Publish the corrected control command
            self.publish_control(usol)

            # Publish markers and reference path
            self.publish_circle_marker()
            self.publish_reference_path()

            # Update state variables
            self.w0 = usol[2]  # Update path progress rate
            self.s0 += self.dt * self.w0  # Update path progress
<<<<<<< HEAD
=======

            # Log the data
            self.log_data(self.current_state, self.s_real, usol_real, self.dt)

>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097
        else:
            self.stop_robot()
            if self.initialized and goal_dist <= 0.15:
                self.save_log_to_csv()  # Save the log when the robot reaches the goal

    def log_data(self, state, s0, usol, time_elapsed):
        """Log the current state, s0, usol, and time elapsed."""
        x, y, theta = state
        v, omega, w = usol
        self.data_log.append([x, y, theta, s0, v, omega, w, time_elapsed])

    def save_log_to_csv(self):
        """Save the logged data to a CSV file."""
        with open(self.log_file_path, mode="w", newline="") as file:
            writer = csv.writer(file)
            # Write the header
            writer.writerow(
                ["x", "y", "theta", "s0", "v", "omega", "w", "time_elapsed"]
            )
            # Write the logged data
            writer.writerows(self.data_log)
        self.get_logger().info(f"Data log saved to {self.log_file_path}")


    def stop_robot(self):
        # Publish zero velocity to stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        print("stopped")

    def dist(self, current, goal):
        return np.sqrt((current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2)

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

    def convert_u(self, usol):
        u = [0, 0, 0]

<<<<<<< HEAD
        u[0] = 0.02 + np.clip(usol[0],0,1)*(0.15-0.02)
        u[1] = np.sign(usol[1])*0.05 + np.clip(usol[1],-0.8,0.8)*(0.15-0.05)
        u[2] = 0.9 * np.clip(usol[2],0,1)
=======
        u[0] = 0.02 + np.clip(usol[0], 0, 1) * (0.15 - 0.02)
        u[1] = np.sign(usol[1]) * 0.05 + np.clip(usol[1], -0.8, 0.8) * (0.15 - 0.05)
        u[2] = 0.95 * np.clip(usol[2], 0, 1)
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

        return u

    def dist(self, current, goal):
        return np.sqrt((current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2)

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

    def save_to_csv(self, filename="src/plan_xy.csv"):
        """Save the logged x, y"""
        with open(filename, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["x", "y","theta"])  # Header
            writer.writerows(self.path_log)
        self.get_logger().info(f"Data saved to {filename}")

    def save_to_csv_replan(self, filename="src/replan_xy.csv"):
        """Save the logged x, y"""
        with open(filename, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["x", "y"])  # Header
            writer.writerows(self.path_log)
        self.get_logger().info(f"Replanned Data saved to {filename}")


def main(args=None):
    rclpy.init(args=args)
    nmpc_controller = NMPCController()

    rclpy.spin(nmpc_controller)

    nmpc_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
