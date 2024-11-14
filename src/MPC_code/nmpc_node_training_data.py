#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, Point
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

# from .functions.path_planner import *


class NMPCController(Node):

    def __init__(self):
        super().__init__('nmpc_controller')

        # NMPC Parameters
        self.Ts = 1  # Sampling time
        self.N = 6  # Prediction horizon
        self.nx = 4  # State dimension (x, y, theta,s)
        self.nu = 3  # Input dimension (v, omega, w)

        self.dt = 0
        self.start = 0
        self.end = 0

        # Bounds
        self.umax = np.array([0.1, 0.1])    # Upper bounds on controls
        self.lb_u = np.array([0, -0.1])   # Lower bounds on controls
        self.ub_u = np.array([0.1, 0.1])    # Upper bounds on controls
        self.lb_w = 0                   # Lower bound for w
        self.ub_w = 1                   # Upper bound for w
        self.lb_s = -0.3                   # Lower bound for s (reference trajectory variable)
        self.ub_s = 0.3

        # Weight matrices for cost function
        self.Q = np.diag([1000, 1000, 0])   # State weight
        self.R = np.diag([1, 1])        # Control input weight
        self.T = 10

        # Other initializations
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ref_path_pub = self.create_publisher(Path, '/ref_path', 10)
        self.ol_path_pub = self.create_publisher(Path, '/ol_path', 10)

        # self.global_path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        self.obs_sub = self.create_subscription(Obstacles, '/obstacles', self.obs_callback, 10)
        self.max_obs = 2
        self.obs_list = np.zeros((self.max_obs, 3))

        self.current_state = np.array([])
        self.goal = []

        self.initialized = False
        self.u0 = np.array([0.1, 0])
        self.w0 = 1
        self.s0 = 0

        # Array to store x, y, theta, s values
        self.data_log = []

        # LiDAR scan subscriber
        self.scan_sub = self.create_subscription(LaserScan, '/altered_scan', self.scan_callback, 10)

        # Setup TF2 listener to transform LiDAR frame coordinates
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.time.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # To store the dynamic LiDAR frame center
        self.lidar_frame_center = None
        self.A = None
        self.b = None

        self.segment_length = 20
        self.epsilon = 0.15
        self.v_max = 0.1

        self.new_goal_received = False  # Flag to track new goal pose

        # Control loop initialization
        self.last_time = None
        self.control_loop_timer = self.create_timer(0.1, self.control_loop)

        # Create a publisher for the marker
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        # Initialize a unique ID for the marker
        self.marker_id = 0

        self.last_transform_update_time = None

        self.ref_path = None


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

    # def path_callback(self, msg):
    #     if self.new_goal_received:
    #         path_points = []
    #         self.data_log = []
    #         for pose in msg.poses:
    #             x = pose.pose.position.x
    #             y = pose.pose.position.y
    #             quaternion = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
    #             _, _, theta = euler_from_quaternion(quaternion)

    #             path_points.append((x, y, theta))
    #             self.data_log.append([x,y])

            # self.global_path = LSPB_fit(np.array(path_points),self.segment_length,self.epsilon,self.v_max)

            # # Define the CasADi variable for s
            # s = ca.MX.sym('s')

            # # Define a CasADi function to evaluate the selected (x, y) for a given s
            # self.ref_path = ca.Function('f_s', [s], [f(self.global_path, s)])

            # fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

            # for i, segment in enumerate(self.global_path):

            #     if segment.segment_type == 'line':
            #         s = np.linspace(segment.start_time, segment.end_time-0.01, 1000)
            #         x_vals = []
            #         y_vals = []

            #         for s_value in s:
            #             result = self.ref_path(s_value)
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
            #             result = self.ref_path(s_value)
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

    def goal_pose_callback(self, msg):
        self.new_goal_received = True

        x = msg.pose.position.x
        y = msg.pose.position.y
        quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        _, _, theta = euler_from_quaternion(quaternion)
        
        self.goal = [x, y, theta]

    def get_base_footprint_transform(self):
        """Get the current transform of 'base_footprint' with respect to 'map'."""
        try:
            # Get the transform between 'map' and 'base_footprint'
            transform = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())

            # Extract the translation
            translation = transform.transform.translation
            x = translation.x
            y = translation.y

            # Extract the rotation quaternion and convert it to euler angles (yaw is the Z-axis rotation)
            rotation = transform.transform.rotation
            quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
            _, _, theta = euler_from_quaternion(quaternion)

            
            return np.array([x, y, theta])

        except Exception as e:
            self.get_logger().warn(f"Could not get transform for base_footprint: {str(e)}")

            return None
        
    def get_base_footprint_transform(self):
        """Get the current transform of 'base_footprint' with respect to 'map'."""
        current_time = self.get_clock().now()
        
        # Update transform only if enough time has passed since the last update
        if self.last_transform_update_time is None or (current_time - self.last_transform_update_time).nanoseconds > 100000000:  # 0.1 seconds
            try:
                transform = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time(), timeout=rclpy.time.Duration(seconds=0.1))
                self.last_transform_update_time = current_time

                # Extract the translation and rotation
                translation = transform.transform.translation
                x = translation.x
                y = translation.y
                rotation = transform.transform.rotation
                quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
                _, _, theta = euler_from_quaternion(quaternion)

                # Create and publish the marker
                marker = Marker()
                marker.header.frame_id = 'odom'
                marker.header.stamp = rclpy.time.Time().to_msg()
                marker.ns = 'robot_pose_marker'
                marker.id = self.marker_id
                marker.type = Marker.SPHERE  # Choose the shape you prefer
                marker.action = Marker.ADD

                # Set the pose of the marker to the robot's global position
                marker.pose.position.x = x
                marker.pose.position.y = y
                # marker.pose.position.z = position.z  # Adjust if you want the marker above the robot
                marker.pose.orientation = transform.transform.rotation

                # Set the scale of the marker
                marker.scale.x = 0.2  # Adjust the size as needed
                marker.scale.y = 0.2
                marker.scale.z = 0.2

                # Set the color of the marker (RGBA)
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0  # Don't forget to set alpha to non-zero!

                # Set the lifetime of the marker
                marker.lifetime = Duration(nanosec=1_000_000_000)  # Marker lasts for 1 second

                # Publish the marker
                self.marker_publisher.publish(marker)

                # Increment marker ID if needed (useful when adding/removing markers)
                self.marker_id += 1

                self.current_state = np.array([x, y, theta])

            except Exception as e:
                # self.get_logger().warn(f"Could not get transform for base_footprint: {str(e)}")
                self.stop_robot()

        return self.current_state

    def gamma(self,eta):
        # Define a small threshold to prevent division by zero
        eta_safe = fmax(eta, 1e-10)  # Prevents eta from being exactly zero

        # Calculate non-zero eta case safely
        sqrt_term = sqrt(1 + (2 * eta_safe) ** 2)
        arcsinh_term = arcsinh(2 * eta_safe) / (2 * eta_safe)
        
        # Combine with if_else, guaranteeing no division by zero
        result = if_else(eta != 1e-10, 0.5 * (sqrt_term + arcsinh_term), 1)
        return result

    def g(self,v_max,eta):
        return v_max/self.gamma(eta)

    def reference_traj(self,s, v_max=1.0, eta=0.5,lbs=-0.2,ubs=0.2):
        result = SX.zeros(3)
        result[0] = self.g(v_max, eta) * s

        y_pb = eta * (self.g(v_max, eta) * s) ** 2

        lbx = self.g(v_max, eta) * lbs
        lby = eta * (self.g(v_max, eta) * lbs) ** 2
        mlb = 2 * eta * self.g(v_max, eta) * lbs

        ylb = mlb*self.g(v_max, eta)*s + lby - mlb * lbx


        mrb = 2 * eta * self.g(v_max, eta) * ubs
        rbx = self.g(v_max, eta) * ubs
        rby = eta * (self.g(v_max, eta) * ubs) ** 2

        yrb = mrb*self.g(v_max, eta)*s + rby - mrb * rbx

        result[1] = if_else(s<lbs,ylb,if_else(s>ubs,yrb,y_pb))

        return result


    def control_loop(self):

        now = self.get_clock().now()

        if self.start == 0:
            self.start = time.perf_counter()
        else:
            self.end = time.perf_counter()
            self.dt = self.end - self.start
            self.start = self.end

            # Get the current state from the transform of 'base_footprint'
        self.current_state = self.get_base_footprint_transform()

        if self.current_state is None:
            self.stop_robot()
            return  # If the transform is unavailable, skip this control loop iteration

        if not self.initialized:
            self.setup_mpc(self.current_state)
            self.publish_reference_path()  # Publish the reference path once initialized

        elif self.initialized and self.dt > 0:

            self.x0 = np.append(self.current_state,np.array([self.s0]))

            self.solver.set(0, "p", np.transpose(self.obs_list).reshape((1, -1))[0].reshape((-1, 1)))

            self.solver.set(0, "lbx", self.x0)
            self.solver.set(0, "ubx", self.x0)

            status = self.solver.solve()

            if status != 0:
                print(f"ACADOS returned status {status}")
                
            usol = self.solver.get(0, "u")

            x_opt = np.array([self.solver.get(i, "x") for i in range(self.ocp.dims.N + 1)])

            self.publish_control(0.1*usol)
            # print(usol)
            self.publish_reference_path()
            self.publish_ol_path(x_opt)

            self.w0 = usol[2]
            self.s0 += self.dt * self.w0

    def stop_robot(self):
        # Publish zero velocity to stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


    def dist(self, current, goal):
        return np.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)

    def publish_ol_path(self, path):
        ol_path = Path()
        ol_path.header.stamp = self.get_clock().now().to_msg()
        ol_path.header.frame_id = "odom"  # Adjust frame_id to your setup

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
        ref_path.header.frame_id = "odom"  # Adjust frame_id to your setup

        for s in np.linspace(self.lb_s, self.ub_s, num=100):
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

    def setup_mpc(self, x0):
        self.x0 = np.append(x0,np.array([self.s0]))
        self.initialized = True

        # Initialize NMPC settings after global path is received and processed
        self.ocp = self.setup_ocp_with_cost_function()

        # Make sure coeff_x and coeff_y are properly flattened and concatenated
        parameter_values = np.transpose(self.obs_list).reshape((1, -1))[0].reshape((-1, 1))

        # Initialize the solver
        self.solver = AcadosOcpSolver(self.ocp, json_file="acados_ocp.json")

        # Set the parameter values in the solver
        self.solver.set(0, "p", parameter_values)

    def get_lidar_frame_transform(self):
        """Manually extract the translation and yaw angle from 'lidar_frame' to 'map'."""
        try:
            # Get the transform between 'map' and 'lidar_frame'
            transform = self.tf_buffer.lookup_transform('odom', 'lidar_frame', rclpy.time.Time())

            # Extract the translation
            translation = transform.transform.translation
            translation_vec = np.array([translation.x, translation.y, translation.z])

            # Extract the rotation quaternion and convert it to euler angles (yaw is the Z-axis rotation)
            rotation = transform.transform.rotation
            quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
            _, _, yaw = euler_from_quaternion(quaternion)

            return translation_vec, yaw

        except Exception as e:
            self.get_logger().warn(f"Could not get transform for lidar_frame: {str(e)}")
            return None, None

    def manually_transform_point(self, point, translation_vec, yaw):
        """Manually transform a point using translation and yaw rotation."""
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        rotation_matrix = np.array([[cos_yaw, -sin_yaw],
                                    [sin_yaw, cos_yaw]])

        # Apply the rotation to the point (assuming z=0, planar motion)
        point_vec = np.array([point.x, point.y])
        rotated_point_vec = np.dot(rotation_matrix, point_vec)

        transformed_point = Point(x=rotated_point_vec[0] + translation_vec[0],
                                  y=rotated_point_vec[1] + translation_vec[1],
                                  z=0.0)
        return transformed_point

    def scan_callback(self, msg):
        try:
            # Get the translation and yaw angle for manual transform
            translation_vec, yaw = self.get_lidar_frame_transform()

            # If the transform isn't available, skip this callback
            if translation_vec is None or yaw is None:
                return

            # Create a point for lidar_frame_center
            lidar_frame_center = Point(x=translation_vec[0], y=translation_vec[1], z=translation_vec[2])

            # Filters for range limits
            min_range = 0.1  # Minimum valid range value (ignore too close points)
            max_range = 10.0  # Maximum valid range value (ignore far points)

            # Process the scan data and manually transform all points first
            angle = msg.angle_min
            transformed_points = []
            for i in range(len(msg.ranges)):
                range_value = msg.ranges[i]

                # Skip invalid points (out of range or zero)
                if range_value < min_range or range_value > max_range:
                    angle += msg.angle_increment
                    continue

                # Calculate the position of the point in the lidar_frame
                x = range_value * math.cos(angle)
                y = range_value * math.sin(angle)
                point = Point(x=x, y=y, z=0.0)

                # Manually transform the point from lidar_frame to map frame
                transformed_point = self.manually_transform_point(point, translation_vec, yaw)

                transformed_points.append(transformed_point)

                # Increment the angle for the next scan point
                angle += msg.angle_increment

            # Now calculate the polygon matrix (Ax + By <= C) for the lines through each pair of points
            self.compute_subshapes_and_plot(lidar_frame_center, transformed_points)

        except Exception as e:
            self.get_logger().error(f"Error processing scan: {str(e)}")
 

    def dist(self,current,goal):
        return np.sqrt((current[0]-goal[0])**2 + (current[1]-goal[1])**2)

    def publish_ol_path(self, path):
        ol_path = Path()
        ol_path.header.stamp = self.get_clock().now().to_msg()
        ol_path.header.frame_id = "odom"  # Adjust frame_id to your setup

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

        h = if_else(obs[2] > 0,fmax(obs[2]**2 - (x[0]-obs[0]) ** 2 - (x[1]-obs[1]) ** 2, 0),0)

        return h

    def mobile_robot_ode(self):
        # Define state variables
        x = SX.sym("x")
        y = SX.sym("y")
        theta = SX.sym("theta")
        s = SX.sym("s")

        # Define control inputs
        v = SX.sym("v")
        omega = SX.sym("omega")
        w = SX.sym("w")

        # Real and artificial states and controls
        states = vertcat(x, y, theta, s)
        controls = vertcat(v, omega, w)

        obs_list = SX.sym("obs", self.max_obs, 3)

        # System dynamics
        dx = v * cos(theta)
        dy = v * sin(theta)
        dtheta = omega
        ds = w

        # Artificial dynamics are free variables (no dynamics for simplicity)
        xdot = vertcat(dx, dy, dtheta, ds)

        # Define the model
        model = AcadosModel()
        model.f_expl_expr = xdot
        model.x = states
        model.u = controls

        model.p = reshape(obs_list, -1, 1)
        model.name = "mobile_robot"

        return model

    def setup_ocp_with_cost_function(self):
        ocp = AcadosOcp()
        model = self.mobile_robot_ode()
        ocp.model = model

        N = 12
        Ts = 0.5
        T = N * Ts

        mu=10

        ocp.dims.N = N
        ocp.solver_options.tf = T

        # Define weights
        Q = np.diag([1000, 1000, 0])  # Adjust the weights as needed for state deviation
        R = np.diag([0.01, 0.01])  # Control effort weights
        T_cost = np.array([10])  # Weight for time dilation cost


        # State variables
        x = ocp.model.x[:3]  # (x, y, theta)
        u = ocp.model.u[:2]  # (v, omega)
        w = ocp.model.u[2]  # w is the third control input
        s = ocp.model.x[3]  # s is the path parameter

        obs_list = reshape(ocp.model.p, self.max_obs, 3)

        # Reference trajectory
        xi_s = self.reference_traj(s)
        dx = x - xi_s

        # Define cost function expressions
        ocp.model.cost_y_expr = vertcat(dx, u, (1 - w))

        # for i in range(self.max_obs):
        #     ocp.model.cost_y_expr = vertcat(ocp.model.cost_y_expr,self.obstacle(x, obs_list[i, :]))

        ocp.model.cost_y_expr_e = vertcat(dx)

        # Ensure the dimensions match the weights
        # ocp.cost.W = block_diag(Q, R, T_cost,0.5*mu*np.diag([1]*self.max_obs))
        ocp.cost.W = block_diag(Q, R, T_cost)
        ocp.cost.W_e = Q

        # Provide an initial reference value for `yref` and `yref_e`
        ny = ocp.model.cost_y_expr.size()[0]
        ocp.cost.yref = np.zeros((ny,))
        ocp.cost.yref_e = np.zeros((dx.size()[0],))

        # Set parameter values (initialize as zeros, these will be updated during execution)
        ocp.parameter_values = np.zeros((self.max_obs*3, 1))

        ocp.constraints.lbu = np.array([0, -1, 0])  # Lower bounds for (v, omega, w)
        ocp.constraints.ubu = np.array([1, 1, 0.1])  # Upper bounds for (v, omega, w)
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
        x0_initial = np.array([0, 0, 0, 0])  # (x, y, theta, s)
        ocp.constraints.x0 = x0_initial

        return ocp

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
