#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, Point, PoseWithCovarianceStamped
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
import pandas as pd


from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

from .functions.path_planner import *

import torch
from .functions.policy_model import PolicyModel

import subprocess
import re


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
        self.cmd_vel_pub = self.create_publisher(
            Twist, "/diffbot_base_controller/cmd_vel_unstamped", 10
        )

        # self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.ref_path_pub = self.create_publisher(Path, "/ref_path", 10)
        self.ol_path_pub = self.create_publisher(Path, "/ol_path", 10)
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.amcl_pose_callback, 10
        )

        self.goal_pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_pose_callback, 10
        )

        # self.obs_sub = self.create_subscription(
        #     Obstacles, "/obstacles", self.obs_callback, 10
        # )
        self.max_obs = 2
        self.obs_list = np.zeros((self.max_obs, 3))

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
        self.marker_publisher = self.create_publisher(
            Marker, "/visualization_marker", 10
        )
        # Initialize a unique ID for the marker
        self.marker_id = 0

        self.last_transform_update_time = None

        self.ref_path = None
        self.global_path = None

        self.old_vel = None

        # self.model = load_model("/home/bertrandt/b.ob/src/nmpc_robot_controller/nmpc_robot_controller/functions/path_following_model.h5")
        # self.model = load_model("/home/ubuntu/b.ob/src/nmpc_robot_controller/nmpc_robot_controller/functions/path_following_model.h5")

        self.pf_model = PolicyModel(
            input_dim=5, output_dim=3
        )  # Replace with your actual architecture
        self.pf_model.load_state_dict(
            torch.load(
                "/home/noorshawaf/NY/s/b.ob/src/MPC_code/IL/DPL/models/final_policy.pth"
            )
        )
        self.pf_model.eval()  # Set the model to evaluation mode

        # Load the CSV data
        self.csv_path = "/home/noorshawaf/NY/s/b.ob/src/nmpc_robot_controller/nmpc_robot_controller/functions/path_data_log_left.csv"  # Replace with the path to your CSV file
        self.path_points = self.load_csv_data(self.csv_path)

        self.data_log = []  # Stores all the logged data
        self.log_file_path = (
            "src/collected_data/closed_loop_nn_pf_rpi.csv"  # Update this path
        )

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

    def load_csv_data(self, file_path):
        # Load CSV using pandas
        df = pd.read_csv(file_path)
        # Convert DataFrame to a list of tuples [(x1, y1), (x2, y2), ...]
        return df[["x", "y"]].values.tolist()

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

    def goal_pose_callback(self, msg):
        # Use the CSV data instead of generating new path points
        path_points = self.path_points  # This uses the loaded CSV data directly

        # Fit the path using LSPB
        self.global_path, points = LSPB_fit(
            np.array(path_points), self.segment_length, self.epsilon, self.v_max
        )

        # Create the reference trajectory function
        s = ca.MX.sym("s")
        self.reference_traj = ca.Function("f_s", [s], [f(self.global_path, s)])

        self.new_goal_received = False
        self.initialized = True

        # Set self.goal to the final endpoint of the global path with theta = 0
        final_position = self.global_path[-1].end_point
        self.goal = np.array([final_position[0], final_position[1], 0.0])

    def predict_policy(self, input_data):
        # Convert the input data to a PyTorch tensor
        input_tensor = torch.tensor(input_data, dtype=torch.float32)
        # Perform the prediction
        with torch.no_grad():
            output = self.pf_model(input_tensor)
        return output.numpy()  # Convert output to NumPy array for further processing

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
            return  # If the transform is unavailable, skip this control loop iteration

        if self.goal.shape[0] > 0:
            goal_dist = self.dist(self.current_state, self.goal)
        else:
            goal_dist = 0

        if self.global_path != None:
            self.publish_reference_path()  # Publish the reference path once initialized

        if self.global_path != None and self.dt > 0 and goal_dist > 0.2:

            x_hat, y_hat, theta_hat, s_hat, eta = T_z(
                self.global_path,
                self.current_state[0],
                self.current_state[1],
                self.current_state[2],
                self.s0,
            )

            # Handle orientation flip for eta < 0
            if eta >= 0:
                input_data = [[x_hat, y_hat, theta_hat, s_hat, eta]]
            else:
                input_data = [[x_hat, -y_hat, -theta_hat, s_hat, -eta]]

            usol = self.predict_policy(input_data)

            # Adjust NN output based on eta
            if eta < 0:
                usol[0, 1] *= -1

            # Clip NN outputs to enforce constraints
            usol = np.clip(usol, [0.01, -0.8, 0.01], [1, 0.8, 1])

            # Pt = 1.0
            # Pn = 1.0

            # en,et,phi = error(self.global_path,self.current_state,self.s0)

            # print(en)
            # print(et)

            # usol[0][0]-= Pt*et
            # usol[0][1]-= Pn*en

            usol = self.convert_u(usol[0])
            usol = self.low_pass_filter(usol, self.old_vel)

            # print(usol)
            self.publish_control(usol)
            # print(usol)
            self.publish_reference_path()

            self.w0 = usol[2]
            self.s0 += self.dt * self.w0

            self.old_vel = usol

            self.log_data(self.current_state, self.s0, usol, self.dt)

        else:
            self.stop_robot()
            if self.initialized and goal_dist <= 0.2:
                self.save_log_to_csv()  # Save the log when the robot reaches the goal

    def log_data(self, state, s0, usol, time_elapsed):
        """Log the current state, s0, usol, and time elapsed."""
        x, y, theta = state
        v, omega, w = usol
        self.data_log.append([x, y, theta, s0, v, omega, w, time_elapsed])

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

    def low_pass_filter(self, usol, old_usol, alpha=0.4):
        """Applies a low-pass filter to smooth the control output."""

        if old_usol is not None:
            usol[0] = alpha * usol[0] + (1 - alpha) * old_usol[0]
            usol[1] = alpha * usol[1] + (1 - alpha) * old_usol[1]
            return usol
        else:
            return usol

    def publish_control(self, control_input):
        twist_msg = Twist()
        twist_msg.linear.x = control_input[0]
        twist_msg.angular.z = control_input[1]
        self.cmd_vel_pub.publish(twist_msg)

    def convert_u(self, usol):
        u = [0, 0, 0]

        u[0] = 0.02 + usol[0] * (0.15 - 0.02)
        u[1] = np.sign(usol[1]) * 0.05 + usol[1] * (0.15 - 0.05)
        u[2] = 0.95 * usol[2]

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

    def save_to_csv(self, filename="path_data_log.csv"):
        """Save the logged x, y, theta, s data to a CSV file."""
        with open(filename, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["x", "y"])  # Header
            writer.writerows(self.data_log)
        self.get_logger().info(f"Data saved to {filename}")

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


def main(args=None):
    rclpy.init(args=args)
    nmpc_controller = NMPCController()

    rclpy.spin(nmpc_controller)

    nmpc_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
