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

<<<<<<< HEAD
import torch
from .functions.policy_model import PolicyModel

import subprocess
import re

=======
import subprocess
import re


>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097
class NMPCController(Node):

    def __init__(self):
        super().__init__("nmpc_controller")

        # NMPC Parameters
        self.Ts = 0.3  # Sampling time
        self.N = 60  # Prediction horizon
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
<<<<<<< HEAD
        # self.cmd_vel_pub = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ref_path_pub = self.create_publisher(Path, '/ref_path', 10)
        self.ol_path_pub = self.create_publisher(Path, '/ol_path', 10)
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.amcl_pose_callback,10)

        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
=======
        # self.cmd_vel_pub = self.create_publisher(
        #     Twist, "/diffbot_base_controller/cmd_vel_unstamped", 10
        # )

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.ref_path_pub = self.create_publisher(Path, "/ref_path", 10)
        self.ol_path_pub = self.create_publisher(Path, "/ol_path", 10)
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.amcl_pose_callback, 10
        )

        self.goal_pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_pose_callback, 10
        )
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

        # self.obs_sub = self.create_subscription(Obstacles, '/obstacles', self.obs_callback, 10)
        self.obs_inflation = 0.0
        self.obs_list = [
            {"s": 15, "d": 0.23, "r": 0.0 + self.obs_inflation},
            {"s": 30, "d": -0.3, "r": 0.0 + self.obs_inflation},
        ]

        # self.obs_inflation = 0.0
        # self.obs_list = [
        #     {"s": 15, "d": 0.2, "r": 0.0 + self.obs_inflation},
        #     {"s": 23, "d": 0.1, "r": 0.0 + self.obs_inflation},
        #     {"s": 30, "d": -0.2, "r": 0.0 + self.obs_inflation},
        # ]

        # The positions (x, y) will be computed later when the goal is set
        for obs in self.obs_list:
            obs["x"], obs["y"] = None, None  # Placeholder for positions

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

<<<<<<< HEAD
        # self.model = load_model("/home/bertrandt/b.ob/src/nmpc_robot_controller/nmpc_robot_controller/functions/path_following_model.h5")
        # self.model = load_model("/home/ubuntu/b.ob/src/nmpc_robot_controller/nmpc_robot_controller/functions/path_following_model.h5")

        self.pf_model = PolicyModel(input_dim=5, output_dim=3)  # Replace with your actual architecture
        self.pf_model.load_state_dict(torch.load("/home/bertrandt/b.ob/src/MPC_code/IL/DPL/models/final_policy.pth"))
        self.pf_model.eval()  # Set the model to evaluation mode

        # Load the CSV data
        self.csv_path = '/home/bertrandt/b.ob/src/nmpc_robot_controller/nmpc_robot_controller/functions/path_data_log_left.csv'  # Replace with the path to your CSV file
        self.path_points = self.load_csv_data(self.csv_path)

        self.data_log = []  # Stores all the logged data
        self.log_file_path = '/home/bertrandt/b.ob/src/MPC_code/path_fitting/closed_loop_nn_pf_1.csv'  # Update this path
=======
        self.old_vel = None

        # Load the CSV data
        self.csv_path = "/home/bertrandt/b.ob/src/nmpc_robot_controller/nmpc_robot_controller/functions/path_data_log_right.csv"  # Replace with the path to your CSV file
        self.path_points = self.load_csv_data(self.csv_path)

        self.obs_position = None  # Replace with your desired position
        self.obs_s = 35
        self.obs_d = 0.58
        self.obs_r = 0.0  # Set the radius

        self.data_log = []  # Stores all the logged data
        self.log_file_path = "src/closed_loop_mpc_pf_right.csv"  # Update this path

        self.saved = False

    def find_closest_obstacle(self):
        """Find the closest obstacle to the current robot position."""
        current_x, current_y = self.current_state[0], self.current_state[1]
        closest_obs = min(
            self.obs_list,
            key=lambda obs: np.sqrt(
                (current_x - obs["x"]) ** 2 + (current_y - obs["y"]) ** 2
            ),
        )
        return np.array([closest_obs["x"], closest_obs["y"], closest_obs["r"]])

    def publish_circle_marker(self, height=0.1, frame_id="map"):
        for obs in self.obs_list:
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacle_marker"
            marker.id = self.marker_id
            self.marker_id += 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = obs["x"]
            marker.pose.position.y = obs["y"]
            marker.pose.position.z = 0.0  # Ground level
            marker.pose.orientation.w = 1.0

            marker.scale.x = 2 * (obs["r"] - self.obs_inflation)  # Diameter
            marker.scale.y = 2 * (obs["r"] - self.obs_inflation)  # Diameter
            marker.scale.z = height  # Thin cylinder

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            self.marker_publisher.publish(marker)
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

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
<<<<<<< HEAD
        return df[['x', 'y']].values.tolist()
=======
        return df[["x", "y"]].values.tolist()
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

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

    def goal_pose_callback(self, msg):
        # Use the CSV data instead of generating new path points
        path_points = self.path_points  # This uses the loaded CSV data directly

<<<<<<< HEAD
    def obs_callback(self, msg):

        # Initialize an array to store obstacle information
        obs = np.zeros((len(msg.circles), 3))

        # Extract the x, y, radius for each circle and store it in the obs array
        for i in range(len(msg.circles)):
            obs[i] = [msg.circles[i].center.x, msg.circles[i].center.y, msg.circles[i].radius]

        # Calculate the difference between the obstacle positions and the current position
        try:
            diff = obs[:, :2] - np.tile(self.current_state[:2], (obs[:, :2].shape[0], 1))

            # Calculate the Euclidean distance
            distances = np.linalg.norm(diff, axis=1)

            # Sort the obs array according to the distances
            sorted_indices = np.argsort(distances)
            obs_sorted = obs[sorted_indices]

            if obs_sorted.shape[0] < self.max_obs:
                obs_sorted = np.vstack((obs_sorted, np.zeros((self.max_obs - obs_sorted.shape[0], 3))))

            self.obs_list = obs_sorted[:self.max_obs, :]
            
        except:
            self.obs_list = np.zeros((self.max_obs, 3))

    def goal_pose_callback(self, msg):
        # Use the CSV data instead of generating new path points
        path_points = self.path_points  # This uses the loaded CSV data directly

        # Fit the path using LSPB
        self.global_path, points = LSPB_fit(np.array(path_points), self.segment_length, self.epsilon, self.v_max)
        
        # Create the reference trajectory function
        s = ca.MX.sym('s')
        self.reference_traj = ca.Function('f_s', [s], [f(self.global_path, s)])

        self.new_goal_received = False
        self.initialized = True
        
=======
        # Fit the path using LSPB
        self.global_path, points = LSPB_fit(
            np.array(path_points), self.segment_length, self.epsilon, self.v_max
        )

        # Create the reference trajectory function
        s = ca.MX.sym("s")
        self.reference_traj = ca.Function("f_s", [s], [f(self.global_path, s)])

        self.ub_s = self.global_path[-1].end_time
        self.new_goal_received = False

>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097
        # Set self.goal to the final endpoint of the global path with theta = 0
        final_position = self.global_path[-1].end_point
        self.goal = np.array([final_position[0], final_position[1], 0.0])

<<<<<<< HEAD

    def predict_policy(self, input_data):
        # Convert the input data to a PyTorch tensor
        input_tensor = torch.tensor(input_data, dtype=torch.float32)
        # Perform the prediction
        with torch.no_grad():
            output = self.pf_model(input_tensor)
        return output.numpy()  # Convert output to NumPy array for further processing
=======
        # Compute the positions of all obstacles based on the global path
        for obs in self.obs_list:
            obs["x"], obs["y"] = get_deviated_point(
                self.global_path, obs["s"], obs["d"]
            )

        # Set the initial obstacle position for visualization
        self.obs_position = np.array(
            [self.obs_list[0]["x"], self.obs_list[0]["y"], 0.0]
        )
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

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

        if not self.initialized and self.global_path != None and self.ub_s != None:
            self.setup_mpc(self.current_state)
            self.publish_reference_path()  # Publish the reference path once initialized

<<<<<<< HEAD
        if self.global_path!=None and self.dt > 0 and goal_dist>0.1:
=======
        if (
            self.initialized
            and self.global_path != None
            and self.dt > 0
            and goal_dist > 0.1
        ):
            self.x0 = np.append(self.current_state, np.array([self.s0]))
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

            # Find the closest obstacle
            closest_obs = self.find_closest_obstacle()

<<<<<<< HEAD
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

            # usol = self.convert_u(usol[0])
            # usol = self.low_pass_filter(usol, self.old_vel)

            # print(usol)
            self.publish_control(usol[0])
            # print(usol)
            self.publish_reference_path()

            self.w0 = usol[0][2]
            self.s0 += self.dt * self.w0

            self.log_data(self.current_state, self.s0, usol[0], self.dt)
            
        else:
            self.stop_robot()
            if self.initialized and goal_dist <= 0.1:
                self.save_log_to_csv()  # Save the log when the robot reaches the goal
=======
            # Set the closest obstacle as the parameter for all stages
            for k in range(self.N):
                self.solver.set(k, "p", closest_obs.flatten())

            self.solver.set(0, "lbx", self.x0)
            self.solver.set(0, "ubx", self.x0)

            status = self.solver.solve()

            if status != 0:
                print(f"ACADOS returned status {status}")
                return

            # Extract control inputs and proceed
            usol = self.solver.get(0, "u")
            x_opt = np.array(
                [self.solver.get(i, "x") for i in range(self.ocp.dims.N + 1)]
            )

            usol_real = usol.copy()
            self.s_real += self.dt * usol[2]

            # usol = self.convert_u(usol)
            # usol = self.low_pass_filter(usol, self.old_vel)

            # Publish results
            self.publish_control(usol)
            self.publish_reference_path()
            self.publish_ol_path(x_opt)

            # Update state and reference parameter
            self.w0 = usol[2]
            self.s0 += self.dt * self.w0

            self.old_vel = usol

            # Log the data
            self.log_data(self.current_state, self.s_real, usol_real, self.dt)

        else:
            if self.initialized and goal_dist <= 0.15 and not self.saved :
                self.stop_robot()
                self.save_log_to_csv()  # Save the log when the robot reaches the goal
                self.saved = True
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

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

<<<<<<< HEAD
    def low_pass_filter(self,usol, old_usol, alpha=0.2):
        """Applies a low-pass filter to smooth the control output."""
        usol[0] = alpha * usol[0] + (1 - alpha) * old_usol[0]
        usol[1] = alpha * usol[1] + (1 - alpha) * old_usol[1]
        return usol
    
=======
        for obs in self.obs_list:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacle_marker"
            marker.id = self.marker_id
            self.marker_id += 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = obs["x"]
            marker.pose.position.y = obs["y"]
            marker.pose.position.z = 0.0  # Ground level
            marker.pose.orientation.w = 1.0

            marker.scale.x = 2 * (obs["r"] - self.obs_inflation)  # Diameter
            marker.scale.y = 2 * (obs["r"] - self.obs_inflation)  # Diameter
            marker.scale.z = 0.1  # Thin cylinder

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            self.marker_publisher.publish(marker)

    def low_pass_filter(self, usol, old_usol, alpha=0.4):
        """Applies a low-pass filter to smooth the control output."""

        if old_usol is not None:
            usol[0] = alpha * usol[0] + (1 - alpha) * old_usol[0]
            usol[1] = alpha * usol[1] + (1 - alpha) * old_usol[1]
            return usol
        else:
            return usol

>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097
    def publish_control(self, control_input):
        twist_msg = Twist()
        twist_msg.linear.x = control_input[0]
        twist_msg.angular.z = control_input[1]
        self.cmd_vel_pub.publish(twist_msg)

    def convert_u(self, usol):
        u = [0, 0, 0]

<<<<<<< HEAD
        u[0] = 0.02 + usol[0]*(0.15-0.02)
        u[1] = np.sign(usol[1])*0.05 + usol[1]*(0.15-0.05)
        u[2] = 0.8 * usol[2]
=======
        u[0] = 0.02 + usol[0] * (0.15 - 0.02)
        u[1] = np.sign(usol[1]) * 0.05 + usol[1] * (0.15 - 0.05)
        u[2] = 0.95 * usol[2]
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

        return u

    def setup_mpc(self, x0):
        self.x0 = np.append(x0, np.array([self.s0]))
        self.initialized = True

        # Initialize NMPC settings after global path is received and processed
        self.ocp = self.setup_ocp_with_cost_function()

        # Use a placeholder for one obstacle initially
        closest_obs = self.find_closest_obstacle()
        parameter_values = closest_obs.flatten()  # Single obstacle (3 parameters)

        # Initialize the solver
        self.solver = AcadosOcpSolver(self.ocp, json_file="acados_ocp.json")

        # Set the placeholder obstacle as the initial parameter
        self.solver.set(0, "p", parameter_values)

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

    def obstacle(self, x, obs):

<<<<<<< HEAD
    def save_log_to_csv(self):
        """Save the logged data to a CSV file."""
        with open(self.log_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Write the header
            writer.writerow(['x', 'y', 'theta', 's0', 'v', 'omega', 'w', 'time_elapsed'])
            # Write the logged data
            writer.writerows(self.data_log)
        self.get_logger().info(f"Data log saved to {self.log_file_path}")

=======
        h = if_else(
            obs[2] > 0,
            fmax(obs[2] ** 2 - (x[0] - obs[0]) ** 2 - (x[1] - obs[1]) ** 2, 0),
            0,
        )

        return h

    def mobile_robot_ode(self):
        x = ca.SX.sym("x")
        y = ca.SX.sym("y")
        theta = ca.SX.sym("theta")
        s = ca.SX.sym("s")
        v = ca.SX.sym("v")
        omega = ca.SX.sym("omega")
        w = ca.SX.sym("w")

        states = ca.vertcat(x, y, theta, s)
        controls = ca.vertcat(v, omega, w)
        obs_list = ca.SX.sym("obs", 1, 3)  # Single obstacle (closest one)

        dx = v * ca.cos(theta)
        dy = v * ca.sin(theta)
        dtheta = omega
        ds = w

        xdot = ca.vertcat(dx, dy, dtheta, ds)

        model = AcadosModel()
        model.f_expl_expr = xdot
        model.x = states
        model.u = controls
        model.p = ca.reshape(obs_list, -1, 1)  # Closest obstacle as parameter
        model.name = "mobile_robot"

        return model

    def setup_ocp_with_cost_function(self):
        ocp = AcadosOcp()
        model = self.mobile_robot_ode()
        ocp.model = model

        N = 60
        Ts = 0.3
        T = N * Ts

        mu = 8 * 10**2

        ocp.dims.N = N
        ocp.solver_options.tf = T

        # Define weights
        Q = np.diag([10, 10, 0])  # State weights (3x3)
        R = np.diag([1, 1])  # Control input weights (2x2)
        T_cost = np.array([[10]])  # Weight for time dilation cost (1x1)

        # State variables
        x = ocp.model.x[:3]  # (x, y, theta)
        u = ocp.model.u[:2]  # (v, omega)
        w = ocp.model.u[2]  # w is the third control input
        s = ocp.model.x[3]  # s is the path parameter

        obs_list = ca.reshape(ocp.model.p, 1, 3)  # Single obstacle as parameter

        # Reference trajectory
        xi_s = self.reference_traj(s)
        dx = x - xi_s

        # Cost function expressions
        ocp.model.cost_y_expr = ca.vertcat(dx, u, (1 - w))
        obs_x = obs_list[0, 0]
        obs_y = obs_list[0, 1]
        obs_r = obs_list[0, 2]

        # Obstacle avoidance penalty
        h = ca.if_else(
            obs_r > 0,
            ca.fmax(obs_r**2 - (x[0] - obs_x) ** 2 - (x[1] - obs_y) ** 2, 0),
            0,
        )
        ocp.model.cost_y_expr = ca.vertcat(ocp.model.cost_y_expr, h)

        ocp.model.cost_y_expr_e = ca.vertcat(dx)

        # Create the weight matrix with correct dimensions
        obstacle_weights = 0.5 * mu

        ocp.cost.W = np.block(
            [
                [Q, np.zeros((3, 2)), np.zeros((3, 1)), np.zeros((3, 1))],
                [np.zeros((2, 3)), R, np.zeros((2, 1)), np.zeros((2, 1))],
                [np.zeros((1, 3)), np.zeros((1, 2)), T_cost, np.zeros((1, 1))],
                [
                    np.zeros((1, 3)),
                    np.zeros((1, 2)),
                    np.zeros((1, 1)),
                    obstacle_weights,
                ],
            ]
        )

        # ocp.cost.W = block_diag(Q, R, T_cost)
        ocp.cost.W_e = Q

        # Provide an initial reference value for `yref` and `yref_e`
        ny = ocp.model.cost_y_expr.size()[0]
        ocp.cost.yref = np.zeros((ny,))
        ocp.cost.yref_e = np.zeros((dx.size()[0],))

        # Constraints setup
        ocp.constraints.lbx = np.array([0])  # Lower bound for `s`
        ocp.constraints.ubx = np.array([self.ub_s])  # Upper bound for `s`
        ocp.constraints.idxbx = np.array([3])  # Index of the constrained state (s)

        # Set parameter values (initialize as zeros, these will be updated during execution)
        ocp.parameter_values = np.zeros((3, 1))

        ocp.constraints.lbu = np.array([0, -0.8, 0])  # Lower bounds for (v, omega, w)
        ocp.constraints.ubu = np.array([1, 0.8, 1])  # Upper bounds for (v, omega, w)
        ocp.constraints.idxbu = np.array([0, 1, 2])

        # Set the cost type to NONLINEAR_LS
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"

        # Solver options
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.tf = T

        # Increase the maximum number of iterations
        ocp.solver_options.nlp_solver_max_iter = 500
        ocp.solver_options.qp_solver_iter_max = 500

        ocp.solver_options.print_level = 0

        # Improve convergence
        ocp.solver_options.regularize_method = "MIRROR"
        ocp.solver_options.levenberg_marquardt = 1e-4

        # Set the initial state directly
        x0_initial = np.append(
            self.current_state, np.array([self.s0])
        )  # (x, y, theta, s)
        ocp.constraints.x0 = x0_initial

        return ocp

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
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097


def main(args=None):
    rclpy.init(args=args)
    nmpc_controller = NMPCController()

    rclpy.spin(nmpc_controller)

    nmpc_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
