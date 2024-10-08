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

from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration


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
        self.lb_s = 0                   # Lower bound for s (reference trajectory variable)

        # Weight matrices for cost function
        self.Q = np.diag([1000, 1000, 0])   # State weight
        self.R = np.diag([1, 1])        # Control input weight
        self.T = 10

        # Other initializations
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ref_path_pub = self.create_publisher(Path, '/ref_path', 10)
        self.ol_path_pub = self.create_publisher(Path, '/ol_path', 10)

        self.global_path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        self.current_state = []
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
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # To store the dynamic LiDAR frame center
        self.lidar_frame_center = None
        self.A = None
        self.b = None

        self.global_path = []
        self.segment_length = 10

        self.new_goal_received = False  # Flag to track new goal pose

        # Control loop initialization
        self.last_time = None
        self.control_loop_timer = self.create_timer(0.0001, self.control_loop)

        # Create a publisher for the marker
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        # Initialize a unique ID for the marker
        self.marker_id = 0

    def path_callback(self, msg):
        if self.new_goal_received:
            path_points = []
            for pose in msg.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                quaternion = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
                _, _, theta = euler_from_quaternion(quaternion)
                path_points.append((x, y, theta))
            self.global_path = np.array(path_points)
            self.ub_s = len(self.global_path) / 18
            self.fit_path_segments()
            self.new_goal_received = False

    def goal_pose_callback(self, msg):
        self.new_goal_received = True

        x = msg.pose.position.x
        y = msg.pose.position.y
        quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        _, _, theta = euler_from_quaternion(quaternion)
        
        self.goal = [x, y, theta]

    def fit_path_segments(self):
        x = self.global_path[:, 0]
        y = self.global_path[:, 1]

        segments = []
        prev_end = None

        # Loop advances by self.segment_length each iteration
        for i in range(0, len(x) - self.segment_length + 1, self.segment_length):
            x_seg = x[i:i + self.segment_length]
            y_seg = y[i:i + self.segment_length]

            if prev_end is not None:
                # Add the last end point to the start of the current segment
                x_seg = np.insert(x_seg, 0, prev_end[0])
                y_seg = np.insert(y_seg, 0, prev_end[1])

            if len(x_seg) > 2:
                # Fit a 2nd-degree polynomial for segments with more than 2 points
                coeff_x = np.polyfit(np.linspace(0, 1, len(x_seg)), x_seg, 2)
                coeff_y = np.polyfit(np.linspace(0, 1, len(y_seg)), y_seg, 2)
                degree = 2
            else:
                # Fit a linear polynomial if there are only 2 points
                coeff_x = np.polyfit([0, 1], x_seg, 1)
                coeff_y = np.polyfit([0, 1], y_seg, 1)
                degree = 1

            s = SX.sym('s')
            x_s = sum([coeff_x[j] * s**(degree - j) for j in range(degree + 1)])
            y_s = sum([coeff_y[j] * s**(degree - j) for j in range(degree + 1)])

            f_x = Function('f_x', [s], [x_s])
            f_y = Function('f_y', [s], [y_s])

            segments.append((f_x, f_y))

            # Update prev_end with the final point of the current segment
            prev_end = (f_x(1).full().flatten()[0], f_y(1).full().flatten()[0])

        # Now add the final segment from the last fitted point to the final goal
        final_goal = (x[-1], y[-1])
        if prev_end is not None and (prev_end[0] != final_goal[0] or prev_end[1] != final_goal[1]):
            x_seg = np.array([prev_end[0], final_goal[0]])
            y_seg = np.array([prev_end[1], final_goal[1]])

            # Fit a linear segment for the final connection
            coeff_x = np.polyfit([0, 1], x_seg, 1)
            coeff_y = np.polyfit([0, 1], y_seg, 1)
            degree = 1

            x_s = coeff_x[0] * s + coeff_x[1]
            y_s = coeff_y[0] * s + coeff_y[1]

            f_x = Function('f_x', [s], [x_s])
            f_y = Function('f_y', [s], [y_s])

            segments.append((f_x, f_y))

        self.fitted_segments = segments

    def reference_traj(self, s):

        scaled_s = s * (len(self.global_path)/self.ub_s) * 1.1

        num_segments = len(self.fitted_segments)
        threshold = 0.5  # Threshold for large changes

        # Determine the current segment index and the normalized segment position
        segment_index = floor(scaled_s / self.segment_length)
        segment_s = fmod(scaled_s, self.segment_length) / self.segment_length

        # Ensure the segment_index is within bounds
        segment_index = fmin(segment_index, num_segments - 1)

        # Initialize the position variables
        eta_x = SX(0)
        eta_y = SX(0)

        for i, (f_x, f_y) in enumerate(self.fitted_segments):
            # Get the value of the current segment
            current_x = f_x(segment_s)
            current_y = f_y(segment_s)

            # Check if the segment is the one to use for this s
            eta_x = if_else(segment_index == i, current_x, eta_x)
            eta_y = if_else(segment_index == i, current_y, eta_y)

            # Check for a large change in x or y
            if i < num_segments - 1:
                next_f_x, next_f_y = self.fitted_segments[i + 1]
                next_x = next_f_x(0)
                next_y = next_f_y(0)

                large_change_x = fabs(next_x - current_x) > threshold
                large_change_y = fabs(next_y - current_y) > threshold

                # Use nested if_else to handle both conditions
                eta_x = if_else(large_change_x, next_f_x(1), eta_x)
                eta_y = if_else(large_change_y, next_f_y(1), eta_y)

        eta = vertcat(eta_x, eta_y, 0.0)
        return eta

    def odom_callback(self, msg):
        now = rclpy.time.Time()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation

        quaternion = [quat.x, quat.y, quat.z, quat.w]

        if len(quaternion) != 4:
            raise Exception("Invalid quaternion received")
        # Convert quaternion to Euler angles (yaw)
        _, _, theta = euler_from_quaternion(quaternion)

        self.current_state = np.array([x, y, theta])

        # Create and publish the marker
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = now.to_msg()
        marker.ns = 'robot_pose_marker'
        marker.id = self.marker_id
        marker.type = Marker.SPHERE  # Choose the shape you prefer
        marker.action = Marker.ADD

        # Set the pose of the marker to the robot's global position
        marker.pose.position.x = x
        marker.pose.position.y = y
        # marker.pose.position.z = position.z  # Adjust if you want the marker above the robot
        marker.pose.orientation = quat

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



    def control_loop(self):

        # Compute time difference (dt) for NMPC
        current_time = time.time()
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        if self.start == 0:
            self.start = time.perf_counter()
        else:
            self.end = time.perf_counter()
            self.dt = self.end - self.start
            self.start = self.end

        if not self.initialized and hasattr(self, 'fitted_segments'):
            self.setup_mpc(self.current_state)
            self.publish_reference_path()  # Publish the reference path once initialized

        elif self.initialized:
            goal_dist = self.dist(self.current_state, self.goal)

            if goal_dist > 0.2:

                self.x0 = np.append(self.current_state,np.array([self.s0]))

                self.solver.set(0, "lbx", self.x0)
                self.solver.set(0, "ubx", self.x0)

                status = self.solver.solve()

                if status != 0:
                    print(f"ACADOS returned status {status}")
                    
                usol = self.solver.get(0, "u")

                x_opt = np.array([self.solver.get(i, "x") for i in range(self.ocp.dims.N + 1)])

                self.publish_control(usol)
                self.publish_reference_path()
                self.publish_ol_path(x_opt)

                self.w0 = usol[2]
                self.s0 += self.dt * self.w0

            else:
                self.stop_robot()

    def stop_robot(self):
        # Publish zero velocity to stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def update_current_state_from_tf(self):
        try:
            # Set a timeout for the transform lookup
            timeout = rclpy.time.Duration(seconds=2.0)  # Set a 2-second timeout

            # Get the transformation between 'map' and 'odom'
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(), timeout)


        except tf2_ros.LookupException:
            self.get_logger().warn("Transform not found from 'map' to 'base_footprint'. Waiting for it...")
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f"Extrapolation error: {str(e)}")
        except Exception as e:
            self.get_logger().warn(f"Error in transform from 'map' to 'base_footprint': {str(e)}")


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
        self.solver = AcadosOcpSolver(self.ocp, json_file="acados_ocp.json")

    def get_lidar_frame_transform(self):
        """Manually extract the translation and yaw angle from 'lidar_frame' to 'map'."""
        try:
            # Get the transform between 'map' and 'lidar_frame'
            transform = self.tf_buffer.lookup_transform('map', 'lidar_frame', rclpy.time.Time())

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

    def publish_reference_path(self):
        ref_path = Path()
        ref_path.header.stamp = self.get_clock().now().to_msg()
        ref_path.header.frame_id = "map"  # Adjust frame_id to your setup

        for s in np.linspace(self.lb_s, self.ub_s, num=100):
            eta_val = self.reference_traj(s)
            pose = PoseStamped()
            pose.header.stamp = ref_path.header.stamp
            pose.header.frame_id = ref_path.header.frame_id
            pose.pose.position.x = float(eta_val[0])
            pose.pose.position.y = float(eta_val[1])
            pose.pose.position.z = 0.0
            # Assume no orientation or flat trajectory, set quaternion accordingly
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            ref_path.poses.append(pose)

        self.ref_path_pub.publish(ref_path)

    def publish_control(self, control_input):
        twist_msg = Twist()
        twist_msg.linear.x = control_input[0]
        twist_msg.angular.z = control_input[1]
        self.cmd_vel_pub.publish(twist_msg)


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
        model.name = "mobile_robot"

        return model

    def setup_ocp_with_cost_function(self):
        ocp = AcadosOcp()
        model = self.mobile_robot_ode()
        ocp.model = model

        N = 6
        Ts = 1
        T = N * Ts

        ocp.dims.N = N
        ocp.solver_options.tf = T

        # Define weights
        Q = np.diag([100, 100, 0])  # Adjust the weights as needed for state deviation
        R = np.diag([1, 1])  # Control effort weights
        T_cost = np.array([15])  # Weight for time dilation cost

        # State variables
        x = ocp.model.x[:3]  # (x, y, theta)
        u = ocp.model.u[:2]  # (v, omega)
        w = ocp.model.u[2]  # w is the third control input
        s = ocp.model.x[3]  # s is the path parameter

        # Reference trajectory
        xi_s = self.reference_traj(s)
        dx = x - xi_s

        # Define cost function expressions
        ocp.model.cost_y_expr = vertcat(dx, u, (1 - w))
        ocp.model.cost_y_expr_e = vertcat(dx)

        # Ensure the dimensions match the weights
        ocp.cost.W = block_diag(Q, R, T_cost)
        ocp.cost.W_e = Q

        # Provide an initial reference value for `yref` and `yref_e`
        ny = ocp.model.cost_y_expr.size()[0]
        ocp.cost.yref = np.zeros((ny,))
        ocp.cost.yref_e = np.zeros((dx.size()[0],))

        # Constraints setup
        ocp.constraints.lbx = np.array([0])  # Lower bound for `s`
        ocp.constraints.ubx = np.array([self.ub_s])  # Upper bound for `s`
        ocp.constraints.idxbx = np.array([3])  # Index of the constrained state (s)

        ocp.constraints.lbu = np.array([0, -0.1, 0])  # Lower bounds for (v, omega, w)
        ocp.constraints.ubu = np.array([0.1, 0.1, 1])  # Upper bounds for (v, omega, w)
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
        x0_initial = np.array([4, 0, 0, 0])  # (x, y, theta, s)
        ocp.constraints.x0 = x0_initial

        return ocp

    def save_to_csv(self, filename='nmpc_data_log.csv'):
        """Save the logged x, y, theta, s data to a CSV file."""
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y', 'theta', 's','obs1_x','obs1_y','obs1_r','obs2_x','obs2_y','obs2_r'])  # Header
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
