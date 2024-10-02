#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
import numpy as np
from casadi import *
import time
import csv  # Import CSV module
import tf2_ros
from tf_transformations import euler_from_quaternion
import math
from sklearn.linear_model import LinearRegression


class NMPCController(Node):

    def __init__(self):
        super().__init__('nmpc_controller')

        # NMPC Parameters
        self.Ts = 1  # Sampling time
        self.lmda = 0.05
        self.N = 8  # Prediction horizon
        self.nx = 3  # State dimension (x, y, theta)
        self.nu = 2  # Input dimension (v, w)

        self.dt = 0
        self.start = 0
        self.end = 0

        # Bounds
        self.umax = np.array([0.08, 0.08])    # Upper bounds on controls
        self.lb_u = np.array([0, -0.08])   # Lower bounds on controls
        self.ub_u = np.array([0.08, 0.08])    # Upper bounds on controls
        self.lb_w = 0                   # Lower bound for w
        self.ub_w = 1                   # Upper bound for w
        self.lb_s = 0                   # Lower bound for s (reference trajectory variable)

        # Weight matrices for cost function
        self.Q = np.diag([100, 100, 0])   # State weight
        self.R = np.diag([1, 1])        # Control input weight
        self.T = 10

        # Other initializations
        self.cmd_vel_pub = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        self.ref_path_pub = self.create_publisher(Path, '/ref_path', 10)
        self.ol_path_pub = self.create_publisher(Path, '/ol_path', 10)

        self.global_path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        self.current_state = []
        self.goal = []

        self.initialized = False
        self.u0 = np.array([0.5, 2])
        self.u0_a = np.array([0.5, 2])
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

        # Create a timer to periodically update the current state from the odom frame
        self.create_timer(0.1, self.update_current_state_from_tf)

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
            self.ub_s = len(self.global_path) / 12
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

    def update_current_state_from_tf(self):
        try:
            # Get the transformation between 'odom' and 'base_link' (replace with appropriate robot frame if different)
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())

            # Extract the translation (x, y)
            translation = transform.transform.translation
            x = translation.x
            y = translation.y

            # Extract the rotation quaternion and convert to Euler angles (yaw)
            rotation = transform.transform.rotation
            quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
            _, _, theta = euler_from_quaternion(quaternion)

            # Update current state with x, y, and theta from odom frame
            self.current_state = np.array([x, y, theta])

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
                    x_pred, usol, w, s = self.run_open_loop_mpc(
                        self.x0, self.s0, self.x_st_0,
                        self.u_st_0, self.w_st_0, self.s_st_0, self.pisolver, 
                        np.transpose(self.A).reshape((1, -1)), np.transpose(self.b).reshape((1, -1))
                    )
                    self.publish_control(usol[0])
                    self.publish_reference_path()
                    self.publish_ol_path(x_pred)
                    self.x0 = self.current_state
                    self.w0 = w[0]
                    self.s0 += self.dt * self.w0

                    self.u_st_0 = np.vstack((usol[1:], usol[-1]))
                    self.x_st_0 = np.vstack((x_pred[1:], x_pred[-1]))
                    self.w_st_0 = np.vstack((w[1:], w[-1]))
                    self.s_st_0 = np.vstack((s[1:], s[-1]))
                else:
                    self.stop_robot()

        except Exception as e:
            self.get_logger().warn(f"Could not get transform from 'odom' to 'base_link': {str(e)}")

    def dist(self, current, goal):
        return np.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

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

    def run_open_loop_mpc(self, x0, s0, x_st_0, u_st_0, w_st_0, s_st_0, solver, A, b):
        args_p = np.append(x0, s0)
        args_p = vertcat(*args_p, *A, *b)

        args_x0 = np.concatenate([
            x_st_0.T.reshape(-1),
            u_st_0.T.reshape(-1),
            w_st_0.T.reshape(-1),
            s_st_0.T.reshape(-1),
        ])
        sol = solver(x0=args_x0, p=args_p, lbg=self.lbg_vcsd, ubg=self.ubg_vcsd)

        x_pred = np.array(sol["x"][: self.nx * (self.N + 1)]).reshape((self.N + 1, self.nx))
        usol = np.array(sol["x"][self.nx * (self.N + 1): self.nx * (self.N + 1) + self.nu * self.N]).reshape((self.N, self.nu))
        w = np.array(sol["x"][self.nx * (self.N + 1) + self.nu * self.N: self.nx * (self.N + 1) + self.nu * self.N + self.N])
        s = np.array(sol["x"][self.nx * (self.N + 1) + self.nu * self.N + self.N:])

        return x_pred, usol, w, s

    def setup_mpc(self, x0):
        self.x0 = x0
        self.u_st_0 = np.tile(self.u0, (self.N, 1))
        self.x_st_0 = np.tile(self.x0, (self.N + 1, 1)).T
        self.w_st_0 = np.tile(self.w0, (self.N, 1))
        self.s_st_0 = np.tile(self.s0, (self.N + 1, 1))
        self.initialized = True

        # Initialize NMPC settings after global path is received and processed
        self.lbg_vcsd, self.ubg_vcsd, self.G_vcsd, self.pisolver = self.Pi_opt_formulation()

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

    def compute_A_b_matrix_from_triangle(self, triangle_points):
        """Compute the A and b matrix for the triangle formed by the given points."""
        A = np.zeros((3, 2))  # Each row will be [A_i, B_i] for the inequality A_i * x + B_i * y <= b_i
        b = np.zeros(3)

        # Loop through each edge of the triangle
        for i in range(3):
            p1 = triangle_points[i]
            p2 = triangle_points[(i + 1) % 3]

            # Compute the normal vector to the edge (p1, p2)
            edge_vector = p2 - p1
            normal_vector = np.array([-edge_vector[1], edge_vector[0]])  # Perpendicular vector to the edge

            # Ensure the normal vector points inward (toward the triangle's center)
            centroid = np.mean(triangle_points, axis=0)
            if np.dot(normal_vector, centroid - p1) > 0:
                normal_vector = -normal_vector

            # Normalize the normal vector
            normal_vector = normal_vector / np.linalg.norm(normal_vector)

            # The inequality is: A_i * x + B_i * y <= b_i
            A[i, :] = normal_vector
            b[i] = np.dot(normal_vector, p1)  # Project p1 onto the normal vector to find b_i

        return A, b

    def accumulate_triangle_inequalities(self, A, b, x_grid, y_grid):
        """Accumulate the inequalities for a given triangle."""
        # Initialize an array to store the combined inequalities
        combined_region = np.ones_like(x_grid, dtype=bool)

        # Combine the inequalities for each side of the triangle
        for i in range(A.shape[0]):
            inequality_region = (A[i, 0] * x_grid + A[i, 1] * y_grid <= b[i])
            combined_region &= inequality_region

        return combined_region

    def compute_subshapes_and_plot(self,lidar_frame_center, transformed_points, group_size=5):
        """Divide points into overlapping groups (e.g., 1-5, 5-10), form a triangle for each group, compute A and b matrices, and plot."""
        
        # Convert lidar_frame_center to a numpy array
        center = np.array([float(lidar_frame_center.x), float(lidar_frame_center.y)])
        
        # Convert transformed_points (ROS Points) to an array of (x, y) coordinates
        points = np.array([[float(p.x), float(p.y)] for p in transformed_points])

        # Process points in overlapping groups of `group_size` to form triangles
        num_points = len(points)
        i = 0

        # Define plot limits dynamically based on the points
        x_min = min(points[:, 0].min(), center[0]) - 1
        x_max = max(points[:, 0].max(), center[0]) + 1
        y_min = min(points[:, 1].min(), center[1]) - 1
        y_max = max(points[:, 1].max(), center[1]) + 1

        # Create a grid of x and y values for plotting inequalities
        x_vals = np.linspace(x_min, x_max, 300)
        y_vals = np.linspace(y_min, y_max, 300)
        x_grid, y_grid = np.meshgrid(x_vals, y_vals)

        # Initialize a combined region to accumulate all inequalities
        full_region = np.zeros_like(x_grid, dtype=bool)

        # Accumulate A and b matrices
        A_total = []
        b_total = []

        while i < num_points - 1:
            # Get the group of consecutive points (for a total of `group_size` points)
            group_end = min(i + group_size, num_points)
            group_points = points[i:group_end]

            # Fit a straight line (line of best fit) to the group points
            X = group_points[:, 0].reshape(-1, 1)  # x-values of scan points
            y = group_points[:, 1]  # y-values of scan points
            reg = LinearRegression().fit(X, y)
            slope = reg.coef_[0]
            intercept = reg.intercept_

            # Create points for the line of best fit
            x_fit = np.array([group_points[0, 0], group_points[-1, 0]])
            y_fit = slope * x_fit + intercept
            line_of_best_fit = np.column_stack((x_fit, y_fit))

            # Define the triangle with the LiDAR center, first scan point, last scan point, and the line of best fit
            triangle_points = np.vstack([center, group_points[0], group_points[-1]])

            # Add the line of best fit as the last edge of the triangle
            triangle_points = np.vstack([triangle_points, line_of_best_fit])

            # Compute the A and b matrix for the triangle
            A, b = self.compute_A_b_matrix_from_triangle(triangle_points[:3])

            # Accumulate the inequalities for this triangle
            full_region |= self.accumulate_triangle_inequalities(A, b, x_grid, y_grid)

            # Add A and b for the current triangle to the total list
            A_total.append(A)
            b_total.append(b)

            # Move to the next group, starting at the last point of the current group (ensure overlap)
            i += group_size - 1  # Move by group_size - 1 to ensure overlap

        current_A = np.vstack(A_total)
        current_b = np.hstack(b_total)
        pad_rows = 270 - current_A.shape[0]

        if pad_rows > 0:
            A_padding = np.zeros((pad_rows, 2))
            b_padding = -1 * np.ones(pad_rows)
            self.A = np.vstack((current_A, A_padding))
            self.b = np.hstack((current_b, b_padding))
        else:
            self.A = current_A
            self.b = current_b

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return roll, pitch, yaw
 

    def dist(self,current,goal):
        return np.sqrt((current[0]-goal[0])**2 + (current[1]-goal[1])**2)
                
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
            self.ub_s = len(self.global_path) / 12
            self.fit_path_segments()
            self.new_goal_received = False

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

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

    def run_open_loop_mpc(self, x0, s0, x_st_0, u_st_0, w_st_0, s_st_0, solver, A,b):

        args_p = np.append(x0, s0)

        args_p = vertcat(*args_p, *A, *b)

        args_x0 = np.concatenate([
            x_st_0.T.reshape(-1),
            u_st_0.T.reshape(-1),
            w_st_0.T.reshape(-1),
            s_st_0.T.reshape(-1),
        ])
        sol = solver(x0=args_x0, p=args_p, lbg=self.lbg_vcsd, ubg=self.ubg_vcsd)

        x_pred = np.array(sol["x"][: self.nx * (self.N + 1)]).reshape((self.N + 1, self.nx))
        usol = np.array(sol["x"][self.nx * (self.N + 1): self.nx * (self.N + 1)+self.nu * self.N ]).reshape((self.N, self.nu))
        w = np.array(sol["x"][self.nx * (self.N + 1)+self.nu * self.N : self.nx * (self.N + 1)+self.nu * self.N + self.N ])
        s = np.array(sol["x"][self.nx * (self.N + 1)+self.nu * self.N + self.N :])

        return x_pred, usol, w, s

    def shift(self, T, t0, x0, u, f):
        st = x0
        con = u[0, :]
        st = self.rk4(f, T, st, con)
        x0 = np.array(st.full()).flatten()
        t0 = t0 + T
        u0 = np.vstack([u[1:], u[-1, :]])
        return t0, x0, u0

    def rk4(self, ode, h, x, u):
        k1 = self.mobile_robot_ode(x, u)
        k2 = self.mobile_robot_ode(x + h / 2 * k1, u)
        k3 = self.mobile_robot_ode(x + h / 2 * k2, u)
        k4 = self.mobile_robot_ode(x + h * k3, u)
        xf = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return xf

    def mobile_robot_ode(self, x, u):
        dx1 = u[0] * cos(x[2])
        dx2 = u[0] * sin(x[2])
        dx3 = u[1]
        dx = vertcat(dx1, dx2, dx3)
        return dx

    def feasibility_cost(self, x_ob, A,b, mu=100):

        V_obs = 1.0

        for i in range(90):

            tmp = fmax((A[3*i,0]*x_ob[0] + A[3*i,1]*x_ob[1] - b[3*i]),0)
            tmp += fmax((A[3*i+1,0]*x_ob[0] + A[3*i+1,1]*x_ob[1] - b[3*i+1]),0)
            tmp += fmax((A[3*i+2,0]*x_ob[0] + A[3*i+2,1]*x_ob[1] - b[3*i+1]),0)

            V_obs *= tmp

        return fmin(mu*V_obs, 10000)
    
    def bilin(self, M, x):
        return mtimes(mtimes(x.T, M), x)

    def objective_cost(self, X, U, W, S,A,b):
        J = 0.0
        for i in range(self.N):
            dx = X[:, i] - self.reference_traj(S[i])
            du = U[:, i] - self.umax
            feas_cost = self.feasibility_cost(X[:2, i], A,b)
            J += self.bilin(self.Q, dx) + self.bilin(self.R, du) + self.bilin(self.T, (1 - W[i])) + feas_cost

        deta_N = X[:, self.N] - self.reference_traj(S[self.N])
        feas_cost = self.feasibility_cost(X[:2, self.N], A,b)
        J += self.bilin(self.Q, deta_N) + feas_cost

        return J

    def equality_constraints(self, X, U, S, W, P_a):
        g = []  # Equality constraints initialization
        g.append(X[:, 0] - P_a[:self.nx])  # Initial state constraint
        g.append(S[0] - P_a[self.nx:])
        for i in range(self.N):
            st = X[:, i]
            cons = U[:, i]
            st_next_euler = self.rk4(self.system, self.Ts, st, cons)
            st_next = X[:, i + 1]
            g.append(st_next - st_next_euler)
            g.append(S[i + 1] - S[i] - self.Ts * W[i])
        return g

    def inequality_constraints(self, X, U, S, W):
        hu = []  # Box constraints on input
        hs = []  # Box constraints on s
        hw = []
        hx = []
        for i in range(self.N):
            hu.append(self.lb_u - U[:, i])
            hu.append(U[:, i] - self.ub_u)
            hs.append(self.lb_s - S[i])
            hs.append(S[i] - self.ub_s)
            hw.append(self.lb_w - W[i])
            hw.append(W[i] - self.ub_w)
        hs.append(S[self.N - 1] - S[self.N] + self.lmda)
        return hx, hu, hs, hw

    def Pi_opt_formulation(self):
        X = SX.sym("X", self.nx, (self.N + 1))  # Decision variables (states)
        U = SX.sym("U", self.nu, self.N)  # Decision variables (controls))
        S = SX.sym("S", self.N + 1, 1)  # Decision variable for ref traj
        W = SX.sym("W", 1, self.N)  # Decision variable
        P_a = SX.sym("P_a", self.nx + 1)  # Initial state parameter

        A = SX.sym("A", 270, 2)
        b = SX.sym("A", 270, 1)

        self.system = Function("sys", [X, U], [self.mobile_robot_ode(X, U)])

        J = self.objective_cost(X, U, W, S,A,b)
        g = self.equality_constraints(X, U, S, W, P_a)
        G = vertcat(*g)

        hx, hu, hs, hw = self.inequality_constraints(X, U, S, W)
        Hs = vertcat(*hs)
        Hu = vertcat(*hu)
        Hw = vertcat(*hw)
        Hx = vertcat(*hx)
        G_vcsd = vertcat(*g, *hx, *hu, *hw, *hs)

        lbg = [0] * G.shape[0] + [-np.inf] * (Hx.shape[0] + Hu.shape[0] + Hs.shape[0] + Hw.shape[0])
        ubg = [0] * G.shape[0] + [0] * (Hx.shape[0] + Hu.shape[0] + Hs.shape[0] + Hw.shape[0])

        lbg_vcsd = vertcat(*lbg)
        ubg_vcsd = vertcat(*ubg)

        Opt_Vars = vertcat(
            reshape(X, -1, 1),
            reshape(U, -1, 1),
            reshape(W, -1, 1),
            reshape(S, -1, 1),
        )

        opts_setting = {
            "ipopt.max_iter": 500,
            "ipopt.print_level": 4,
            "print_time": 1,
            "ipopt.acceptable_tol": 1e-6,
            "ipopt.acceptable_obj_change_tol": 1e-6,
        }

        vnlp_prob = {
            "f": J,
            "x": Opt_Vars,
            "p": vertcat(P_a,reshape(A, -1, 1),reshape(b, -1, 1)),
            "g": G_vcsd,
        }
        pisolver = nlpsol("vsolver", "ipopt", vnlp_prob, opts_setting)

        return lbg_vcsd, ubg_vcsd, G_vcsd, pisolver

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
