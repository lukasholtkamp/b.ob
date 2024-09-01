#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from obstacle_detector.msg import Obstacles
import numpy as np
from casadi import *
import time 
from tf_transformations import euler_from_quaternion

class NMPCController(Node):

    def __init__(self):
        super().__init__('nmpc_controller')
        # NMPC Parameters
        self.Ts = 1  # Sampling time
        self.lmda = 0.05
        self.N = 6  # Prediction horizon
        self.nx = 3  # State dimension (x, y, theta)
        self.nu = 2  # Input dimension (v, w)

        self.dt = 0
        self.start = 0
        self.end = 0

        # Bounds
        self.umax = np.array([0.3, 0.2])    # Upper bounds on controls
        self.lb_u = np.array([0, -0.2])   # Lower bounds on controls
        self.ub_u = np.array([0.3, 0.2])    # Upper bounds on controls
        self.lb_w = 0                   # Lower bound for w
        self.ub_w = 1                   # Upper bound for w
        self.lb_s = 0                   # Lower bound for s (reference trajectory variable)

        # Weight matrices for cost function
        self.Q = np.diag([10, 10, 10])   # State weight
        self.K = np.diag([10, 10, 0]) # Artificial state weight
        self.R = np.diag([10, 10])        # Control input weight
        self.S = np.diag([0.01, 0.01])        # Artificial control input weight
        self.T = 3

        # Other initializations
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.ref_path_pub = self.create_publisher(Path, '/ref_path', 10)
        self.ol_path_pub = self.create_publisher(Path, '/ol_path', 10)

        self.obs_sub = self.create_subscription(Obstacles, '/obstacles', self.obs_callback, 10)
        self.global_path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        self.max_obs = 5
        self.obs_list = np.zeros((self.max_obs, 3))
        self.current_state = np.zeros((1, 3))

        self.global_path = []
        self.segment_length = 10

        self.initialized = False
        self.u0 = np.array([0.5, 2])
        self.u0_a = np.array([0.5, 2])
        self.w0 = 1
        self.s0 = 0

        self.new_goal_received = False  # Flag to track new goal pose

    def path_callback(self, msg):
        if self.new_goal_received:
            # Initialize an empty list to hold the (x, y, theta) tuples
            path_points = []

            # Iterate through each pose in the path
            for pose in msg.poses:
                # Extract the x and y coordinates
                x = pose.pose.position.x
                y = pose.pose.position.y

                # Extract the orientation quaternion
                orientation_q = pose.pose.orientation
                quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

                # Convert quaternion to euler (roll, pitch, yaw)
                _, _, theta = euler_from_quaternion(quaternion)

                # Append the (x, y, theta) as a tuple to the list
                path_points.append((x, y, theta))

            # Convert the list of tuples to a NumPy array
            self.global_path = np.array(path_points)

            # Update self.ub_s to be the total number of points in the global_path
            self.ub_s = len(self.global_path) / 12

            # Fit the global path with straight lines and parabolas
            self.fit_path_segments()

            # Reset the flag after processing the new path
            self.new_goal_received = False

    def goal_pose_callback(self, msg):
        # Set the flag to indicate a new goal has been received
        self.new_goal_received = True

    def fit_segment(self, x_seg, y_seg, start_point=None):
        if start_point is not None:
            x_seg = np.insert(x_seg, 0, start_point[0])
            y_seg = np.insert(y_seg, 0, start_point[1])

        if len(x_seg) > 2:
            coeff_x = np.polyfit(np.linspace(0, 1, len(x_seg)), x_seg, 2)
            coeff_y = np.polyfit(np.linspace(0, 1, len(y_seg)), y_seg, 2)
            degree = 2
        else:
            coeff_x = np.polyfit([0, 1], x_seg, 1)
            coeff_y = np.polyfit([0, 1], y_seg, 1)
            degree = 1

        s = SX.sym('s')
        x_s = sum([coeff_x[i] * s**(degree - i) for i in range(degree + 1)])
        y_s = sum([coeff_y[i] * s**(degree - i) for i in range(degree + 1)])

        f_x = Function('f_x', [s], [x_s])
        f_y = Function('f_y', [s], [y_s])

        return f_x, f_y

    def divide_into_segments(self):
        segments = []
        for i in range(0, len(self.global_path) - self.segment_length + 1, self.segment_length):
            segments.append(self.global_path[i:i + self.segment_length])
        return segments

    def fit_path_segments(self):
        x = self.global_path[:, 0]
        y = self.global_path[:, 1]

        segments = []
        prev_end = None

        for i in range(0, len(x) - self.segment_length + 1, self.segment_length):
            x_seg = x[i:i + self.segment_length]
            y_seg = y[i:i + self.segment_length]

            if prev_end is not None:
                x_seg = np.insert(x_seg, 0, prev_end[0])
                y_seg = np.insert(y_seg, 0, prev_end[1])

            if len(x_seg) > 2:
                coeff_x = np.polyfit(np.linspace(0, 1, len(x_seg)), x_seg, 2)
                coeff_y = np.polyfit(np.linspace(0, 1, len(y_seg)), y_seg, 2)
                degree = 2
            else:
                coeff_x = np.polyfit([0, 1], x_seg, 1)
                coeff_y = np.polyfit([0, 1], y_seg, 1)
                degree = 1

            s = SX.sym('s')
            x_s = sum([coeff_x[j] * s**(degree - j) for j in range(degree + 1)])
            y_s = sum([coeff_y[j] * s**(degree - j) for j in range(degree + 1)])

            f_x = Function('f_x', [s], [x_s])
            f_y = Function('f_y', [s], [y_s])

            segments.append((f_x, f_y))

            prev_end = (f_x(1).full().flatten()[0], f_y(1).full().flatten()[0])

        # Now add the final segment from the last fitted point to the final goal
        final_goal = (x[-1], y[-1])
        if prev_end is not None and (prev_end[0] != final_goal[0] or prev_end[1] != final_goal[1]):
            x_seg = np.array([prev_end[0], final_goal[0]])
            y_seg = np.array([prev_end[1], final_goal[1]])

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


    # The rest of your NMPCController class code remains unchanged...

    def setup_mpc(self, x0, x0_a):
        self.x0 = x0
        self.x0_a = x0_a
        self.u_st_0 = np.tile(self.u0, (self.N, 1))
        self.u_st_0_a = np.tile(self.u0_a, (self.N, 1))
        self.x_st_0 = np.tile(self.x0, (self.N + 1, 1)).T
        self.x_st_0_a = np.tile(self.x0_a, (self.N + 1, 1)).T
        self.w_st_0 = np.tile(self.w0, (self.N, 1))
        self.s_st_0 = np.tile(self.s0, (self.N + 1, 1))
        self.initialized = True

        # Initialize NMPC settings after global path is received and processed
        self.lbg_vcsd, self.ubg_vcsd, self.G_vcsd, self.pisolver = self.Pi_opt_formulation()

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

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        _, _, theta = self.quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)

        self.current_state = np.array([x, y, theta])

        if self.start == 0:
            self.start = time.perf_counter()
        else:
            self.end = time.perf_counter()
            self.dt = self.end - self.start
            self.start = self.end

        if not self.initialized and hasattr(self, 'fitted_segments'):
            self.setup_mpc(self.current_state, self.current_state)
            self.publish_reference_path()  # Publish the reference path once initialized
        elif self.initialized:
            x_pred, x_pred_a, usol, usol_a, w, s = self.run_open_loop_mpc(
                self.x0, self.s0, self.x_st_0, self.x_st_0_a,
                self.u_st_0, self.u_st_0_a, self.w_st_0, self.s_st_0, self.pisolver, np.transpose(self.obs_list).reshape((1, -1))
            )
            if self.s0 < self.ub_s - 0.5:  # Avoid overshooting the maximum s
                self.publish_control(usol[0])
                self.publish_reference_path()
                self.publish_ol_path(x_pred)
                self.x0 = self.current_state
                self.w0 = w[0]
                self.s0 += self.dt * self.w0

                self.u_st_0 = np.vstack((usol[1:], usol[-1]))
                self.u_st_0_a = np.vstack((usol_a[1:], usol_a[-1]))
                self.x_st_0 = np.vstack((x_pred[1:], x_pred[-1]))
                self.x_st_0_a = np.vstack((x_pred_a[1:], x_pred_a[-1]))
                self.w_st_0 = np.vstack((w[1:], w[-1]))
                self.s_st_0 = np.vstack((s[1:], s[-1]))
            else:
                self.stop_robot()

    def obs_callback(self, msg):
        # Initialize an array to store obstacle information
        obs = np.zeros((len(msg.circles), 3))

        # Extract the x, y, radius for each circle and store it in the obs array
        for i in range(len(msg.circles)):
            obs[i] = [msg.circles[i].center.x, msg.circles[i].center.y, msg.circles[i].radius]

        diff = obs[:, :2]

        while np.tile(self.current_state[:2], (obs[:, :2].shape[0], 1)).shape[1]!=obs[:, :2].shape[1]:
            try:
                diff = obs[:, :2] - np.tile(self.current_state[:2], (obs[:, :2].shape[0], 1))
            except:
                self.get_logger().info("Waiting for obstacle detection")  # Debugging line
        
        # Calculate the Euclidean distance
        distances = np.linalg.norm(diff, axis=1)
        
        # Sort the obs array according to the distances
        sorted_indices = np.argsort(distances)
        obs_sorted = obs[sorted_indices]

        if obs_sorted.shape[0] < self.max_obs:
            obs_sorted = np.vstack((obs_sorted, np.zeros((self.max_obs - obs_sorted.shape[0], 3))))

        self.obs_list = obs_sorted[:self.max_obs, :]

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

        for s in np.linspace(self.lb_s,self.ub_s,num=100):
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

    def run_open_loop_mpc(self, x0, s0, x_st_0, x_st_0_a, u_st_0, u_st_0_a, w_st_0, s_st_0, solver, obs):

        args_p = np.append(x0, s0)
        args_p = vertcat(*args_p, *obs)
        
        args_x0 = np.concatenate([
            x_st_0.T.reshape(-1),
            x_st_0_a.T.reshape(-1),
            u_st_0.T.reshape(-1),
            u_st_0_a.T.reshape(-1),
            w_st_0.T.reshape(-1),
            s_st_0.T.reshape(-1),
        ])
        sol = solver(x0=args_x0, p=args_p, lbg=self.lbg_vcsd, ubg=self.ubg_vcsd)

        x_pred = np.array(sol["x"][: self.nx * (self.N + 1)]).reshape((self.N + 1, self.nx))
        x_pred_a = np.array(sol["x"][self.nx * (self.N + 1): 2 * self.nx * (self.N + 1)]).reshape((self.N + 1, self.nx))
        usol = np.array(sol["x"][2 * self.nx * (self.N + 1): self.nu * self.N + 2 * self.nx * (self.N + 1)]).reshape((self.N, self.nu))
        usol_a = np.array(sol["x"][self.nu * self.N + 2 * self.nx * (self.N + 1): self.nu * self.N + 2 * self.nx * (self.N + 1) + self.N * self.nu]).reshape((self.N, self.nu))
        w = np.array(sol["x"][self.nu * self.N + 2 * self.nx * (self.N + 1) + self.N * self.nu: self.nu * self.N + 2 * self.nx * (self.N + 1) + self.N * self.nu + self.N])
        s = np.array(sol["x"][self.nu * self.N + 2 * self.nx * (self.N + 1) + self.N * self.nu + self.N:])

        return x_pred, x_pred_a, usol, usol_a, w, s

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

    def obstacle(self, x, obs):
        h = fmax(obs[2]**2 - (x[0]-obs[0]) ** 2 - (x[1]-obs[1]) ** 2, 0)
        return h

    def obstacle_cost(self, x_ob, x_ob_a, obs_list, mu=10):

        V_obs = 0

        for i in range(self.max_obs):

            h_ob = self.obstacle(x_ob, obs_list[i, :])
            h_ob_a = self.obstacle(x_ob_a, obs_list[i, :])

            cost = if_else(obs_list[i, 2] > 0, 0.5 * mu * h_ob**2 + 0.5 * mu * h_ob_a**2, 0)

            V_obs += cost

        return V_obs

    def bilin(self, M, x):
        return mtimes(mtimes(x.T, M), x)


    def objective_cost(self, X, U, W, S_a, X_a, U_a, obs_list):
        J = 0.0
        for i in range(self.N):
            dx = X[:, i] - X_a[:, i]
            du = U[:, i] - U_a[:, i]
            dx_a = X_a[:, i] - self.reference_traj(S_a[i])
            du_a = U_a[:, i] - self.umax
            obs_cost = self.obstacle_cost(X[:2, i], X_a[:2, i], obs_list)
            J += self.bilin(self.Q, dx) + self.bilin(self.R, du) + self.bilin(self.T, (1 - W[i])) + self.bilin(self.K, dx_a) + self.bilin(self.S, du_a) + obs_cost

        deta_Na = X_a[:, self.N] - self.reference_traj(S_a[self.N])
        deta_N = X[:, self.N] - X_a[:, self.N]
        obs_cost = self.obstacle_cost(X[:2, self.N], X_a[:2, self.N], obs_list)
        J += self.bilin(self.Q, deta_N) + self.bilin(self.K, deta_Na) + obs_cost

        return J


    def equality_constraints(self, X, U, S_a, W, P_a, X_a):
        g = []  # Equality constraints initialization
        g.append(X[:, 0] - P_a[:self.nx])  # Initial state constraint
        g.append(X_a[:, 0] - P_a[:self.nx])  # Initial artificial state constraint
        g.append(S_a[0] - P_a[self.nx:])
        for i in range(self.N):
            st = X[:, i]
            cons = U[:, i]
            st_next_euler = self.rk4(self.system, self.Ts, st, cons)
            st_next = X[:, i + 1]
            g.append(st_next - st_next_euler)
            g.append(S_a[i + 1] - S_a[i] - self.Ts * W[i])
        return g

    def inequality_constraints(self, X, U, S_a, W, U_a):
        hu = []  # Box constraints on input
        hu_a = []  # Box constraints on artificial input
        hs = []  # Box constraints on s
        hw = []
        hx = []
        hx_a = []
        for i in range(self.N):
            hu.append(self.lb_u - U[:, i])
            hu.append(U[:, i] - self.ub_u)
            hu_a.append(self.lb_u - U_a[:, i])
            hu_a.append(U_a[:, i] - self.ub_u)
            hs.append(self.lb_s - S_a[i])
            hs.append(S_a[i] - self.ub_s)
            hw.append(self.lb_w - W[i])
            hw.append(W[i] - self.ub_w)
        hs.append(S_a[self.N - 1] - S_a[self.N] + self.lmda)
        return hx, hx_a, hu, hu_a, hs, hw

    def Pi_opt_formulation(self):
        X = SX.sym("X", self.nx, (self.N + 1))  # Decision variables (states)
        X_a = SX.sym("X", self.nx, (self.N + 1))  # Decision variables (artificial states)
        U = SX.sym("U", self.nu, self.N)  # Decision variables (controls)
        U_a = SX.sym("U_a", self.nu, self.N)  # Decision variables (artificial controls)
        S_a = SX.sym("S_a", self.N + 1, 1)  # Decision variable for ref traj
        W = SX.sym("W", 1, self.N)  # Decision variable
        P_a = SX.sym("P_a", self.nx + 1)  # Initial state parameter
        obs_list = SX.sym("obs", self.max_obs, 3)

        self.system = Function("sys", [X, U], [self.mobile_robot_ode(X, U)])

        J = self.objective_cost(X, U, W, S_a, X_a, U_a, obs_list)
        g = self.equality_constraints(X, U, S_a, W, P_a, X_a)
        G = vertcat(*g)

        hx, hx_a, hu, hu_a, hs, hw = self.inequality_constraints(X, U, S_a, W, U_a)
        Hs = vertcat(*hs)
        Hu = vertcat(*hu, *hu_a)
        Hw = vertcat(*hw)
        G_vcsd = vertcat(*g, *hx, *hx_a, *hu, *hu_a, *hw, *hs)

        lbg = [0] * G.shape[0] + [-np.inf] * (Hu.shape[0] + Hs.shape[0] + Hw.shape[0])
        ubg = [0] * G.shape[0] + [0] * (Hu.shape[0] + Hs.shape[0] + Hw.shape[0])

        lbg_vcsd = vertcat(*lbg)
        ubg_vcsd = vertcat(*ubg)
        
        Opt_Vars = vertcat(
            reshape(X, -1, 1),
            reshape(X_a, -1, 1),
            reshape(U, -1, 1),
            reshape(U_a, -1, 1),
            reshape(W, -1, 1),
            reshape(S_a, -1, 1),
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
            "p": vertcat(P_a, reshape(obs_list, -1, 1)),
            "g": G_vcsd,
        }
        pisolver = nlpsol("vsolver", "ipopt", vnlp_prob, opts_setting)

        return lbg_vcsd, ubg_vcsd, G_vcsd, pisolver

def main(args=None):
    rclpy.init(args=args)
    nmpc_controller = NMPCController()

    rclpy.spin(nmpc_controller)

    nmpc_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
