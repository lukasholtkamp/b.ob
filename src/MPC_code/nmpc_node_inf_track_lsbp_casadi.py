#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
import numpy as np
import casadi as ca
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


class NMPCController(Node):

    def __init__(self):
        super().__init__('nmpc_controller')

       # NMPC Parameters
        self.Ts = 0.3  # Sampling time
        self.lmda = 0.05
        self.N = 20  # Prediction horizon
        self.nx = 3  # State dimension (x, y, theta)
        self.nu = 2  # Input dimension (v, w)

        self.dt = 0
        self.start = 0
        self.end = 0

        # Bounds
        self.umax = np.array([1.0, 1.0])    # Upper bounds on controls
        self.lb_u = np.array([0, -1.0])   # Lower bounds on controls
        self.ub_u = np.array([1.0, 1.0])    # Upper bounds on controls
        self.lb_w = 0                   # Lower bound for w
        self.ub_w = 1                   # Upper bound for w
        self.lb_s = 0                   # Lower bound for s (reference trajectory variable)
        self.ub_s = None                  # Upper bound for s (reference trajectory variable)

        # Weight matrices for cost function
        self.Q = np.diag([1000, 1000, 0])   # State weight
        self.K = np.diag([0.5, 0.5, 0]) # Artificial state weight
        self.R = np.diag([0.01, 0.01])        # Control input weight
        self.S = np.diag([0.01, 0.01])        # Artificial control input weight
        self.T = 10

        # Other initializations
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.ref_path_pub = self.create_publisher(Path, '/ref_path', 10)
        self.proper_ref_path_pub = self.create_publisher(Path, '/proper_ref_path', 10)

        self.ol_path_pub = self.create_publisher(Path, '/ol_path', 10)

        self.obs_sub = self.create_subscription(Obstacles, '/obstacles', self.obs_callback, 10)

        self.max_obs = 2
        self.obs_list = np.zeros((self.max_obs, 3))
        self.current_state = np.array([])

        self.goal = None
        self.new_goal_received = False  # Flag to track new goal pose

        self.initialized = False
        self.u0 = np.array([1, 0])
        self.u0_a = np.array([1, 0])
        self.w0 = 1
        self.s0 = 0

        # Array to store x, y, theta, s values
        self.data_log = []

        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        self.segment_length = 55
        self.epsilon = 0.6
        self.v_max = 0.4

        self.control_loop_timer = self.create_timer(0.05, self.control_loop)

        # Create a publisher for the marker
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        # Initialize a unique ID for the marker
        self.marker_id = 0

        self.ref_path = None
        self.global_path = None

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

    def inf_path(self, s):
        T = 90
        etat1 = 6 * np.cos((2 * np.pi / T) * s)
        etat2 = 3.2 * np.sin((4 * np.pi / T) * s)
        eta = (etat1, etat2, 0)
        return eta


    def goal_pose_callback(self, msg):

        path_points = []
        n= 1000
        for i in range(n):

            path_points.append(self.inf_path((i/n)*90))
            # self.data_log.append([x,y])

        self.global_path, points = LSPB_fit(np.array(path_points),self.segment_length,self.epsilon,self.v_max)
        # self.save_to_csv()
        s = ca.MX.sym('s')
        self.ub_s = self.global_path[-1].end_time
        self.reference_traj = ca.Function('f_s', [s], [f(self.global_path, s)])
        self.new_goal_received = False

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        _, _, theta = self.quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)

        self.current_state = np.array([x, y, theta])

    def control_loop(self):

        now = self.get_clock().now()

        if self.start == 0:
            self.start = time.perf_counter()
        else:
            self.end = time.perf_counter()
            self.dt = self.end - self.start
            self.start = self.end

        if self.current_state is None:
            self.stop_robot()
            return  # If the transform is unavailable, skip this control loop iteration

        if not self.initialized and self.ub_s!=None:
            self.setup_mpc(self.current_state, self.current_state)
            self.publish_reference_path()  # Publish the reference path once initialized
            # self.publish_proper_reference_path()

        elif self.initialized and self.dt > 0 and self.s0 < self.ub_s-0.2:

            x_pred, x_pred_a, usol, usol_a, w, s = self.run_open_loop_mpc(
                self.x0, self.s0, self.x_st_0, self.x_st_0_a,
                self.u_st_0, self.u_st_0_a, self.w_st_0, self.s_st_0, self.pisolver, np.transpose(self.obs_list).reshape((1, -1))
            )

            u_sol = [float(usol[0][0]),float(usol[0][1]),float(w[0])]

            u_sol = self.convert_u(u_sol)

            self.publish_control(u_sol)
            self.publish_reference_path()
            # self.publish_proper_reference_path()
            self.publish_ol_path(x_pred)
            self.x0 = self.current_state
            self.w0 = u_sol[2]
            self.s0 += 0.05 * u_sol[2]

            self.u_st_0 = np.vstack((usol[1:], usol[-1]))
            self.u_st_0_a = np.vstack((usol_a[1:], usol_a[-1]))
            self.x_st_0 = np.vstack((x_pred[1:], x_pred[-1]))
            self.x_st_0_a = np.vstack((x_pred_a[1:], x_pred_a[-1]))
            self.w_st_0 = np.vstack((w[1:], w[-1]))
            self.s_st_0 = np.vstack((s[1:], s[-1]))
            
        else:
            self.stop_robot()


    def convert_u(self,usol):
        u = [0,0,0]
        # u[0] = 0.02 + usol[0]*(0.15-0.02)
        # u[1] = 0.05 + usol[1]*(0.15-0.05)
        # u[2] = 0.5 * usol[2]

        u[0] = 0.02 + usol[0]*(0.5-0.02)
        u[1] = 0.05 + usol[1]*(0.5-0.05)
        u[2] = 0.7*usol[2]

        return u
    
    def obs_callback(self, msg):

        # Initialize an array to store obstacle information
        obs = np.zeros((len(msg.circles), 3))

        # Extract the x, y, radius for each circle and store it in the obs array
        for i in range(len(msg.circles)):
            obs[i] = [msg.circles[i].center.x, msg.circles[i].center.y, msg.circles[i].radius]

        # Calculate the difference between the obstacle positions and the current position
        diff = obs[:, :2] - np.tile(self.current_state[:2], (obs[:, :2].shape[0], 1))

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
            # Assume no orientation or flat trajectory, set quaternion accordingly
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            ref_path.poses.append(pose)

        self.ref_path_pub.publish(ref_path)
    
    def inf_proper_path(self, s):
        T = 90
        etat1 = 6 * np.cos((2 * np.pi / T) * s)
        etat2 = 3 * np.sin((4 * np.pi / T) * s)
        eta = (etat1, etat2, 0)
        return eta

    def publish_proper_reference_path(self):
        ref_path = Path()
        ref_path.header.stamp = self.get_clock().now().to_msg()
        ref_path.header.frame_id = "odom"  # Adjust frame_id to your setup

        for s in np.linspace(0, 90, num=1000):
            eta_val = self.inf_proper_path(s)
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

        self.proper_ref_path_pub.publish(ref_path)

    def publish_control(self, control_input):
        twist_msg = Twist()
        twist_msg.linear.x = control_input[0]
        twist_msg.angular.z = control_input[1]
        self.cmd_vel_pub.publish(twist_msg)

    def run_open_loop_mpc(self, x0, s0, x_st_0, x_st_0_a, u_st_0, u_st_0_a, w_st_0, s_st_0, solver, obs):

        args_p = np.append(x0, s0)

        args_p = ca.vertcat(*args_p, *obs)

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
        dx1 = u[0] * ca.cos(x[2])
        dx2 = u[0] * ca.sin(x[2])
        dx3 = u[1]
        dx = ca.vertcat(dx1, dx2, dx3)  # Ensure this is a CasADi object
        return dx

    def obstacle(self, x, obs):
        h = ca.fmax(obs[2]**2 - (x[0]-obs[0]) ** 2 - (x[1]-obs[1]) ** 2, 0)
        return h

    def obstacle_cost(self, x_ob, x_ob_a, obs_list, mu=5):

        V_obs = 0

        for i in range(self.max_obs):

            h_ob = self.obstacle(x_ob, obs_list[i, :])
            h_ob_a = self.obstacle(x_ob_a, obs_list[i, :])

            cost = ca.if_else(obs_list[i, 2] > 0, 0.5 * mu * h_ob**2 + 0.5 * mu * h_ob_a**2, 0)

            V_obs += cost

        return V_obs

    def bilin(self, M, x):
        return ca.mtimes(ca.mtimes(x.T, M), x)

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
        X = ca.SX.sym("X", self.nx, (self.N + 1))  # Decision variables (states)
        X_a = ca.SX.sym("X", self.nx, (self.N + 1))  # Decision variables (artificial states)
        U = ca.SX.sym("U", self.nu, self.N)  # Decision variables (controls)
        U_a = ca.SX.sym("U_a", self.nu, self.N)  # Decision variables (artificial controls)
        S_a = ca.SX.sym("S_a", self.N + 1, 1)  # Decision variable for ref traj
        W = ca.SX.sym("W", 1, self.N)  # Decision variable
        P_a = ca.SX.sym("P_a", self.nx + 1)  # Initial state parameter
        obs_list = ca.SX.sym("obs", self.max_obs, 3)

        self.system = ca.Function("sys", [X, U], [self.mobile_robot_ode(X, U)])

        J = self.objective_cost(X, U, W, S_a, X_a, U_a, obs_list)
        g = self.equality_constraints(X, U, S_a, W, P_a, X_a)
        G = ca.vertcat(*g)

        hx, hx_a, hu, hu_a, hs, hw = self.inequality_constraints(X, U, S_a, W, U_a)
        Hs = ca.vertcat(*hs)
        Hu = ca.vertcat(*hu, *hu_a)
        Hw = ca.vertcat(*hw)
        G_vcsd = ca.vertcat(*g, *hx, *hx_a, *hu, *hu_a, *hw, *hs)

        lbg = [0] * G.shape[0] + [-np.inf] * (Hu.shape[0] + Hs.shape[0] + Hw.shape[0])
        ubg = [0] * G.shape[0] + [0] * (Hu.shape[0] + Hs.shape[0] + Hw.shape[0])

        lbg_vcsd = ca.vertcat(*lbg)
        ubg_vcsd = ca.vertcat(*ubg)

        Opt_Vars = ca.vertcat(
            ca.reshape(X, -1, 1),
            ca.reshape(X_a, -1, 1),
            ca.reshape(U, -1, 1),
            ca.reshape(U_a, -1, 1),
            ca.reshape(W, -1, 1),
            ca.reshape(S_a, -1, 1),
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
            "p": ca.vertcat(P_a, ca.reshape(obs_list, -1, 1)),
            "g": G_vcsd,
        }
        pisolver = ca.nlpsol("vsolver", "ipopt", vnlp_prob, opts_setting)

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
