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
        self.N = 20  # Prediction horizon
        self.nx = 4  # State dimension (x, y, theta,s)
        self.nu = 3  # Input dimension (v, omega, w)

        self.dt = 0
        self.start = 0
        self.end = 0

        # Bounds
        self.umax = np.array([1, 1])    # Upper bounds on controls
        self.lb_u = np.array([0, -1])   # Lower bounds on controls
        self.ub_u = np.array([1, 1])    # Upper bounds on controls
        self.lb_w = 0                   # Lower bound for w
        self.ub_w = 1                   # Upper bound for w
        self.lb_s = 0                   # Lower bound for s (reference trajectory variable)
        self.ub_s = None

        # Weight matrices for cost function
        self.Q = np.diag([1000, 1000, 0])   # State weight
        self.R = np.diag([0.01, 0.01])        # Control input weight
        self.T = 10

        # Other initializations
        self.cmd_vel_pub = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        self.ref_path_pub = self.create_publisher(Path, '/ref_path', 10)
        self.ol_path_pub = self.create_publisher(Path, '/ol_path', 10)
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.amcl_pose_callback,10)

        self.global_path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        self.obs_sub = self.create_subscription(Obstacles, '/obstacles', self.obs_callback, 10)
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
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        # Initialize a unique ID for the marker
        self.marker_id = 0

        self.last_transform_update_time = None

        self.ref_path = None
        self.global_path = None

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

    def path_callback(self, msg):
        if self.new_goal_received:
            path_points = []
            self.data_log = []
            for pose in msg.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                quaternion = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
                _, _, theta = euler_from_quaternion(quaternion)

                path_points.append((x, y, theta))
                self.data_log.append([x,y])

            self.global_path, points = LSPB_fit(np.array(path_points),self.segment_length,self.epsilon,self.v_max)
            self.ub_s = self.global_path[-1].end_time
            # self.save_to_csv()
            s = ca.MX.sym('s')
            self.reference_traj = ca.Function('f_s', [s], [f(self.global_path, s)])
            self.new_goal_received = False

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

    def goal_pose_callback(self, msg):
        self.new_goal_received = True

        x = msg.pose.position.x
        y = msg.pose.position.y
        quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        _, _, theta = euler_from_quaternion(quaternion)
        
        self.goal = np.array([x, y, theta])
    

    def gamma(self,eta):
        # Define a small threshold to prevent division by zero
        eta_safe = ca.fmax(eta, 1e-10)  # Prevents eta from being exactly zero

        # Calculate non-zero eta case safely
        sqrt_term = ca.sqrt(1 + (2 * eta_safe) ** 2)
        arcsinh_term = ca.arcsinh(2 * eta_safe) / (2 * eta_safe)
        
        # Combine with if_else, guaranteeing no division by zero
        result = ca.if_else(eta != 1e-10, 0.5 * (sqrt_term + arcsinh_term), 1)
        return result

    def g(self,v_max,eta):
        return v_max/self.gamma(eta)


    def control_loop(self):

        now = self.get_clock().now()

        if self.start == 0:
            self.start = time.perf_counter()
        else:
            self.end = time.perf_counter()
            self.dt = self.end - self.start
            self.start = self.end

        if self.current_state.shape[0]<=0:
            self.stop_robot()
            return  # If the transform is unavailable, skip this control loop iteration

        if self.goal.shape[0]>0:
            goal_dist = self.dist(self.current_state,self.goal)
        else:
            goal_dist = 0

        if not self.initialized and self.global_path!=None and self.ub_s!=None:
            self.setup_mpc(self.current_state)
            self.publish_reference_path()  # Publish the reference path once initialized

        elif self.initialized and self.dt > 0 and goal_dist>0.2:

            self.x0 = np.append(self.current_state,np.array([self.s0]))

            self.solver.set(0, "p", np.transpose(self.obs_list).reshape((1, -1))[0].reshape((-1, 1)))

            self.solver.set(0, "lbx", self.x0)
            self.solver.set(0, "ubx", self.x0)

            status = self.solver.solve()

            if status != 0:
                print(f"ACADOS returned status {status}")
                
            usol = self.solver.get(0, "u")

            x_opt = np.array([self.solver.get(i, "x") for i in range(self.ocp.dims.N + 1)])

            print(usol)

            usol = self.convert_u(usol)

            print(usol)

            print("\n")

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
        print("stopped")


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

    def convert_u(self,usol):
        u = [0,0,0]

        u[0] = 0.02 + usol[0]*(0.15-0.02)
        u[1] = np.sign(usol[1])*0.05 + usol[1]*(0.15-0.05)
        u[2] = 0.8 * usol[2]

        return u

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
        dx = v * ca.cos(theta)
        dy = v * ca.sin(theta)
        dtheta = omega
        ds = w

        # Artificial dynamics are free variables (no dynamics for simplicity)
        xdot = vertcat(dx, dy, dtheta, ds)

        # Define the model
        model = AcadosModel()
        model.f_expl_expr = xdot
        model.x = states
        model.u = controls

        model.p = ca.reshape(obs_list, -1, 1)
        model.name = "mobile_robot"

        return model

    def setup_ocp_with_cost_function(self):
        ocp = AcadosOcp()
        model = self.mobile_robot_ode()
        ocp.model = model

        N = 20
        Ts = 0.3
        T = N * Ts

        mu=5*10**5

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

        obs_list = ca.reshape(ocp.model.p, self.max_obs, 3)

        # Reference trajectory
        xi_s = self.reference_traj(s)
        dx = x - xi_s

        # Define cost function expressions
        ocp.model.cost_y_expr = vertcat(dx, u, (1 - w))

        for i in range(self.max_obs):
            ocp.model.cost_y_expr = vertcat(ocp.model.cost_y_expr,self.obstacle(x, obs_list[i, :]))

        ocp.model.cost_y_expr_e = vertcat(dx)

        # Ensure the dimensions match the weights
        ocp.cost.W = block_diag(Q, R, T_cost,0.5*mu*np.diag([1]*self.max_obs))
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
        ocp.parameter_values = np.zeros((self.max_obs*3, 1))

        ocp.constraints.lbu = np.array([0, -0.7, 0])  # Lower bounds for (v, omega, w)
        ocp.constraints.ubu = np.array([1, 0.7, 1])  # Upper bounds for (v, omega, w)
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
        x0_initial = np.append(self.current_state,np.array([self.s0]))  # (x, y, theta, s)
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
