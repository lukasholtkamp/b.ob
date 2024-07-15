# Optimization modules
import casadi as ca
import casadi.tools
# Standard python modules
import time
import math as m
import numpy as np
from struct import *
import pandas as pd
import numpy.matlib
import matplotlib.pyplot as plt
import matplotlib.animation as anim
# ROS2 specific modules
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

def shift(T, x0, u_sol):
    f_value = mapping_func(x0, u_sol[0, :].T)
    st = x0 + (T * f_value).full().flatten()
    u_sol = np.append(u_sol[1:, :], u_sol[-1, :], axis=0)
    return st, u_sol

def plt_fnc(state, predict, goal, t, u_cl):
    plt.figure(1)
    plt.grid()
    plt.text(goal[0] - 0.15, goal[1] - 0.2, 'Goal', style='oblique', fontsize=10)
    plt.text(-0.1, 0.15, 'Start', style='oblique', fontsize=10)
    plt.plot(state.T[:, 0], state.T[:, 1])
    plt.plot(0, 0, 'bo')
    plt.plot(goal[0], goal[1], 'ro')
    plt.xlabel('X-position [Meters]')
    plt.ylabel('Y-position [Meters]')
    plt.title('MPC in python')

    fig, (ax1, ax2) = plt.subplots(2)
    fig.suptitle('Control Signals From MPC Solution')
    ax1.plot(t, u_cl[:, 0])
    ax1.grid()
    ax2.plot(t, u_cl[:, 1])
    ax2.grid()
    ax2.set_ylabel('Angular Velocity [m/s]')
    ax2.set_xlabel('Time [s]')
    ax1.set_ylabel('Linear Velocity [rad/s]')
    ax1.set_xlabel('Time [s]')

    plt.show()
    return state, predict, goal, t

# MPC Parameters
Ts = 0.1  # Timestep
N = 40  # Horizon

# Robot Parameters
rob_diameter = 0.54
v_max = 1  # m/s
v_min = -v_max
w_max = ca.pi / 4  # rad/s
w_min = -w_max
acc_v_max = 0.4  # m/ss
acc_w_max = ca.pi / 4  # rad/ss

# System Model
x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')
states = ca.vertcat(x, y, theta)
n_states = 3  # len([states])

# Control system
v = ca.SX.sym('v')
omega = ca.SX.sym('omega')
controls = ca.vertcat(v, omega)
n_controls = 2  # len([controls])

rhs = ca.vertcat(v * ca.cos(theta), v * ca.sin(theta), omega)

# System setup for casadi
mapping_func = ca.Function('f', [states, controls], [rhs])

# Declare empty system matrices
U = ca.SX.sym('U', n_controls, N)

# Parameters:initial state(x0), reference state (xref)
P = ca.SX.sym('P', n_states + n_states)

X = ca.SX.sym('X', n_states, N + 1)

X[:, 0] = P[0:n_states]

for k in range(0, N):
    st = X[:, k]
    con = U[:, k]
    f_value = mapping_func(st, con)
    st_next = st + (Ts * f_value)
    X[:, k + 1] = st_next

ff = ca.Function('ff', [U, P], [X])

# Objective function
obj = 0
g = []

# Weight matrices
Q = np.zeros((3, 3))
Q[0, 0] = 5
Q[1, 1] = 5
Q[2, 2] = 0.1

R = np.zeros((2, 2))
R[0, 0] = 0.5
R[1, 1] = 0.05

for k in range(0, N):
    st = X[:, k]
    con = U[:, k]
    obj = obj + ca.mtimes(ca.mtimes((st - P[3:6]).T, Q), (st - P[3:6])) + ca.mtimes(ca.mtimes(con.T, R), con)
    g = ca.vertcat(g, X[:, k + 1] - (st + Ts * mapping_func(st, con)))  # Dynamic constraints

nlp_prob = {'f': obj, 'x': ca.reshape(U, -1, 1), 'g': g, 'p': P}
opts = {'ipopt.print_level': 0, 'ipopt.tol': 1e-5, 'print_time': False}
solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

# Constraints
lbg = np.zeros((N * n_states, 1))
ubg = np.zeros((N * n_states, 1))

lbw = np.zeros((N * n_controls, 1))
ubw = np.zeros((N * n_controls, 1))

for i in range(N):
    lbw[i * n_controls] = v_min
    lbw[i * n_controls + 1] = w_min
    ubw[i * n_controls] = v_max
    ubw[i * n_controls + 1] = w_max

# Initialization
t0 = 0
x0 = np.array([0.0, 0.0, 0.0])
xref = np.array([4.0, 4.0, 0.0])
u0 = np.zeros((N, 2))
X0 = np.matlib.repmat(x0, N + 1, 1)
sim_time = 20

class MPCPlanner(Node):
    def __init__(self):
        super().__init__('mpc_planner')
        self.vel_pub = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        self.pose_sub = self.create_subscription(Odometry, '/diffbot_base_controller/odom', self.callback_odom, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.callback_goal, 10)  # Goal subscriber

        self.declare_parameter('timer_period', 0.1)
        self.timer_period = self.get_parameter('timer_period').value
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.current_pose = None
        self.current_goal = None

        self.mpciter = 0
        self.predicted_state = []
        self.u_cl = []
        self.t = []
        self.x0 = np.array([0.0, 0.0, 0.0])
        self.xref = np.array([4.0, 4.0, 0.0])
        self.u0 = np.zeros((N, 2))
        self.X0 = np.matlib.repmat(self.x0, N + 1, 1)
        self.t0 = 0

    def callback_odom(self, msg):
        self.current_pose = msg.pose.pose

    def callback_goal(self, msg):
        self.current_goal = msg.pose.position

    def timer_callback(self):
        if self.current_pose is None or self.current_goal is None:
            return

        self.x0 = np.array([self.current_pose.position.x, self.current_pose.position.y, 0.0])
        self.xref = np.array([self.current_goal.x, self.current_goal.y, 0.0])

        P = np.concatenate((self.x0, self.xref))

        while np.linalg.norm(self.x0 - self.xref) > 1e-2 and self.mpciter - sim_time / Ts < 0.0:
            t1 = time.time()

            sol = solver(x0=self.u0.flatten(), p=P, lbg=lbg, ubg=ubg, lbx=lbw, ubx=ubw)
            u_sol = ca.reshape(sol['x'], n_controls, N)

            self.u_cl = np.append(self.u_cl, np.array(u_sol[:, 0]).T)
            self.t = np.append(self.t, self.t0)

            self.x0, self.u0 = shift(Ts, self.x0, u_sol.T)

            self.predicted_state = np.append(self.predicted_state, np.array(self.x0).T)
            self.mpciter += 1

            vel_cmd = Twist()
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 1.0
            self.vel_pub.publish(vel_cmd)

            self.t0 += Ts
            t2 = time.time()
            print('Current state:', self.x0, 'Time:', t2 - t1)

        # print('Prediction:', self.predicted_state)

def main(args=None):
    rclpy.init(args=args)
    mpc_planner = MPCPlanner()
    rclpy.spin(mpc_planner)
    mpc_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
