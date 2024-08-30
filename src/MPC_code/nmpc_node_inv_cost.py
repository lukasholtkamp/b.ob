#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import numpy as np
from casadi import *
import time
import matplotlib.pyplot as plt
import math

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

        self.nscan = 360
        self.obs_x = np.zeros(self.nscan)
        self.obs_y = np.zeros(self.nscan)
        self.lidar_angles = np.linspace(-3.0, 3.0, num=self.nscan)
        self.min_threshold = 0.5
        self.close_r_tuned = 0.001
        self.far_r_tuned = 100.0
        self.safetyRadius = 0.6

        self.best_v = 0.0
        self.best_w = 0.0
        self.obs_close = False
        self.prevOptimalU = [0,0]
        self.v = 0
        self.omega = 0

        # Bounds
        self.umax = np.array([1.0, 1.0])    # Upper bounds on controls
        self.lb_u = np.array([0, -1.0])   # Lower bounds on controls
        self.ub_u = np.array([1.0, 1.0])    # Upper bounds on controls
        self.lb_w = 0                   # Lower bound for w
        self.ub_w = 1                   # Upper bound for w
        self.lb_s = 0                   # Lower bound for s (reference trajectory variable)
        self.ub_s = 60                  # Upper bound for s (reference trajectory variable)

        # Weight matrices for cost function
        self.Q = np.diag([10, 10, 0])   # State weight
        self.K = np.diag([2.5, 2.5, 0]) # Artificial state weight
        self.R = np.diag([1, 1])        # Control input weight
        self.S = np.diag([5, 5])        # Artificial control input weight
        self.T = 15

        # Other initializations
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.ref_path_pub = self.create_publisher(Path, '/ref_path', 10)
        self.ol_path_pub = self.create_publisher(Path, '/ol_path', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.initialized = False
        self.u0 = np.array([0.5, 2])
        self.u0_a = np.array([0.5, 2])
        self.w0 = 1
        self.s0 = [0]

        self.mpc_counter = 0
        self.cost = []

        # Initialize NMPC settings
        self.lbg_vcsd, self.ubg_vcsd, self.G_vcsd, self.pisolver = self.Pi_opt_formulation()

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

        current_state = np.array([x, y, theta])

        if self.start==0:
            self.start = time.perf_counter()
        else:
            self.end = time.perf_counter()
            self.dt = self.end - self.start
            self.start = self.end
        
        self.v = self.prevOptimalU[0]
        self.omega = self.prevOptimalU[1]

        if not self.initialized:
            self.setup_mpc(current_state, current_state)
            self.publish_reference_path()  # Publish the reference path once initialized
        else:
            x_pred, x_pred_a, usol, usol_a, w, s = self.run_open_loop_mpc(
                self.x0, self.s0, self.x_st_0, self.x_st_0_a,
                self.u_st_0, self.u_st_0_a, self.w_st_0, self.s_st_0, self.pisolver,self.obs_x,self.obs_y)
            
           
            if(not self.obs_close):
                self.v = usol[0][0]
                self.omega = usol[0][1]

                self.publish_ol_path(x_pred)
                self.u_st_0 = np.vstack((usol[1:], usol[-1]))
                self.u_st_0_a = np.vstack((usol_a[1:], usol_a[-1]))
                self.x_st_0 = np.vstack((x_pred[1:], x_pred[-1]))
                self.x_st_0_a = np.vstack((x_pred_a[1:], x_pred_a[-1]))
                self.w_st_0 = np.vstack((w[1:], w[-1]))
                self.s_st_0 = np.vstack((s[1:], s[-1]))
            
            elif s[0]>=60:
                self.v = 0.0
                self.omega = 0.0

            self.publish_control([self.v,self.omega])

            self.x0 = current_state
            self.w0 = w[0]
            self.s0 += self.dt * self.w0

            self.best_v = self.v
            self.best_w = self.omega
            

            self.mpc_counter+=1

            if self.mpc_counter>=420:
                plt.figure(figsize=(10, 5))
                plt.clf()
                plt.plot([float(cost.full().flatten()) for cost in self.cost], label='Cost', marker='o')
                plt.xlabel('MPC Iteration')
                plt.ylabel('Cost')
                plt.legend()
                plt.grid(True)
                plt.show()
            

    def scan_callback(self, msg):

        obsx = []
        obsy = []

        min_dist = 1e10
        max_range = msg.range_max

        
        for i in range(len(msg.ranges)):

            ran = msg.ranges[i]

            if (self.min_threshold <= ran and ran <= max_range):
                r = ran
                
                self.prevOptimalU[0] = self.best_v 
                self.prevOptimalU[1] = self.best_w
                self.obs_close = False

            elif (math.isfinite(ran) and ran < self.min_threshold):
                r = self.close_r_tuned
                self.prevOptimalU[0] = 0.0
                self.prevOptimalU[1] = 0.0
                self.obs_close = True

                self.v = 0.0
                self.omega = 0.5

            elif (not math.isfinite(ran) and ran < 0):
                r = 0.0

            elif (not math.isfinite(ran) and ran > 0):
                r = self.far_r_tuned
                self.prevOptimalU[0] = self.best_v 
                self.prevOptimalU[1] = self.best_w

            elif (math.isnan(ran)):
                continue
            else:
                continue
            

            angle = self.lidar_angles[i]
            x = r * np.cos(angle)
            y = r * np.sin(angle)
    
            obsx.append(x)
            obsy.append(y)

        self.obs_x = np.array(obsx)
        self.obs_y = np.array(obsy)

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
    
    def publish_ol_path(self,path):
        ol_path = Path()
        ol_path.header.stamp = self.get_clock().now().to_msg()
        ol_path.header.frame_id = "odom"  # Adjust frame_id to your setup

        for i in range(path.shape[0]):
            val = path[i,:]
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

    def publish_control(self, control_input):
        twist_msg = Twist()
        twist_msg.linear.x = control_input[0]
        twist_msg.angular.z = control_input[1]
        self.cmd_vel_pub.publish(twist_msg)

    def run_open_loop_mpc(self, x0, s0, x_st_0, x_st_0_a, u_st_0, u_st_0_a, w_st_0, s_st_0, solver,obsx,obsy):
        args_p = np.concatenate((x0,np.array(s0),obsx,obsy),axis=0) # ranges

        args_p = vertcat(*args_p)
        args_x0 = np.concatenate([
            x_st_0.T.reshape(-1),
            x_st_0_a.T.reshape(-1),
            u_st_0.T.reshape(-1),
            u_st_0_a.T.reshape(-1),
            w_st_0.T.reshape(-1),
            s_st_0.T.reshape(-1),
        ])
        sol = solver(x0=args_x0, p=args_p, lbg=self.lbg_vcsd, ubg=self.ubg_vcsd)

        self.cost.append(sol["f"])

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

    def reference_traj(self, s):
        T = 60
        etat1 = 6 * cos((2 * pi / T) * s)
        etat2 = 3 * sin((4 * pi / T) * s)
        eta = vertcat(etat1, etat2, 0)
        return eta

    def obstacle(self, x, r, theta): # scan
        scanx = r * np.cos(theta)
        scany = r * np.sin(theta)

        h = fmax(0.2**2-(x[0]-scanx) ** 2 - (x[1]-scany) ** 2 , 0)
        return h

    def distance(self,x,y,obsx,obsy):
        return (x-obsx) ** 2 + (y-obsy) ** 2

    def obstacle_cost(self, x_ob, x_ob_a,obs_x,obs_y, mu=10):

        V_obs = 0
        min_dist = 1e6

        for i in range(obs_x.shape[0]):
            dist = self.distance(x_ob[0],x_ob[1],obs_x[i],obs_y[i])
            min_dist = fmin(min_dist,dist)

        # h_ob = self.obstacle(x_ob,r,theta)
        # h_ob_a = self.obstacle(x_ob_a,r,theta)

        V_obs = if_else(min_dist<=self.safetyRadius,1e6, 1.0 / min_dist)
        
        return V_obs

    def bilin(self, M, x):
        return mtimes(mtimes(x.T, M), x)

    def objective_cost(self, X, U, W, S_a, X_a, U_a,obs_x,obs_y):
        J = 0.0
        for i in range(self.N):
            dx = X[:, i] - X_a[:, i]
            du = U[:, i] - U_a[:, i]
            dx_a = X_a[:, i] - self.reference_traj(S_a[i])
            du_a = U_a[:, i] - self.umax
            obs_cost = self.obstacle_cost(X[:2, i], X_a[:2, i], obs_x,obs_y)
            J += self.bilin(self.Q, dx) + self.bilin(self.R, du) + self.bilin(self.T, (1 - W[i])) + self.bilin(self.K, dx_a) + self.bilin(self.S, du_a) + obs_cost
        deta_Na = X_a[:, self.N] - self.reference_traj(S_a[self.N])
        deta_N = X[:, self.N] - X_a[:, self.N]
        obs_cost = self.obstacle_cost(X[:2, self.N], X_a[:2, self.N], obs_x,obs_y)
        J += self.bilin(self.Q, deta_N) + self.bilin(self.K, deta_Na) + obs_cost

        return J

    def equality_constraints(self, X, U, S_a, W, P_a,X_a):
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
        g.append(X[:, self.N]-X_a[:, self.N])
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
        obs_x = SX.sym("O_x", self.nscan)  # Lidar scans
        obs_y = SX.sym("O-y", self.nscan)  # Lidar scans

        self.system = Function("sys", [X, U], [self.mobile_robot_ode(X, U)])

        J = self.objective_cost(X, U, W, S_a, X_a, U_a,obs_x,obs_y)
        g = self.equality_constraints(X, U, S_a, W, P_a,X_a)
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
            "p": vertcat(P_a,obs_x,obs_y),
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
