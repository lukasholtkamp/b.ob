
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import casadi as ca
import random
import control as ct
import csv
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

from matplotlib.patches import Circle

from waypoint_filter import *
from line_fitting import *
from transform import *
from path_segments import *
from plotting import *

def LSPB_fit(n,epsilon,v_max=0.1):

    file_path = '/home/bertrandt/b.ob/src/MPC_code/path_fitting/path_data_log_2.csv'

    # Get the selected waypoints and their original indices
    original_indices, selected_waypoints, waypoints = select_waypoints_with_indices(file_path, n)

    # Get the indices and the processed waypoints
    processed_indices, processed_waypoints = preprocess_segments_by_deviation(selected_waypoints, original_indices)

    path_segments = []

    # Initialize previous right epsilon and its time
    x_eps_right_prev, y_eps_right_prev = None, None

    if len(processed_indices)==2:

        k = 0
        s_p = processed_waypoints[0]
        f_p = processed_waypoints[1]

        if (f_p[0]-s_p[0]) == 0:
            m = 10**6
        else:
            m = (f_p[1]-s_p[1])/(f_p[0]-s_p[0])

        while processed_indices[1] - processed_indices[0] - n*k > n:
            k+=1

            next_x = waypoints[processed_indices[0]+n*k]

            if np.abs(m)<50:
                y = m* next_x[0] + s_p[1]- m*s_p[0]
                next_p = (next_x[0],y)

            else:
                x = (1/m)*next_x[1] + s_p[0] - (1/m)* s_p[1]
                next_p = (x,next_x[1])

            path_segment = PathSegment(
            start_point=s_p, 
            end_point=next_p, 
            segment_type='line',
            )

            path_segments.append(path_segment)

            s_p = next_p
        
        path_segment = PathSegment(
            start_point=s_p, 
            end_point=(f_p[0], f_p[1]), 
            segment_type='line',
            )

        path_segments.append(path_segment)

    else:
        # Loop through the selected waypoints and create path segments
        for i in range(1, len(processed_waypoints) - 1):

            k = 0
            
            p1 = processed_waypoints[i - 1]
            p2 = processed_waypoints[i]
            p3 = processed_waypoints[i + 1]

            # Calculate epsilon points
            x_eps_left, y_eps_left, x_eps_right, y_eps_right = calculate_epsilon_points_on_line(p1, p2, p3, epsilon, epsilon)

            # If it's the first iteration, create the first segment from p1 to the left epsilon point (straight line)
            if x_eps_right_prev is None:

                if processed_indices[i] - processed_indices[i-1] > n:

                    s_p = p1

                    if (x_eps_left-s_p[0]) == 0:
                        m = 10**6
                    else:
                        m = (y_eps_left-s_p[1])/(x_eps_left-s_p[0])

                    while processed_indices[i] - processed_indices[i-1] - n*k > n:
                        k+=1

                        next_x = waypoints[processed_indices[i-1]+n*k]

                        if np.abs(m)<50:
                            y = m* next_x[0] + s_p[1]- m*s_p[0]
                            next_p = (next_x[0],y)

                        else:
                            x = (1/m)*next_x[1] + s_p[0] - (1/m)* s_p[1]
                            next_p = (x,next_x[1])

                        path_segment = PathSegment(
                        start_point=s_p, 
                        end_point=next_p, 
                        segment_type='line',
                        )

                        path_segments.append(path_segment)

                        s_p = next_p
                    
                    path_segment = PathSegment(
                        start_point=s_p, 
                        end_point=(x_eps_left, y_eps_left), 
                        segment_type='line',
                        )

                    path_segments.append(path_segment)


                else:
                    path_segment = PathSegment(
                        start_point=p1, 
                        end_point=(x_eps_left, y_eps_left), 
                        segment_type='line',
                    )
                    path_segments.append(path_segment)
            
            # Add the straight line segment from the previous right epsilon point to the current left epsilon point
            if x_eps_right_prev is not None:

                if processed_indices[i] - processed_indices[i-1] > n:

                    s_p = (x_eps_right_prev, y_eps_right_prev)

                    if (x_eps_left-s_p[0]) == 0:
                        m = 10**6
                    else:
                        m = (y_eps_left-s_p[1])/(x_eps_left-s_p[0])

                    while processed_indices[i] - processed_indices[i-1] - n*k > n:
                        k+=1

                        next_x = waypoints[processed_indices[i-1]+n*k]

                        if np.abs(m)<50:
                            y = m* next_x[0] + s_p[1]- m*s_p[0]
                            next_p = (next_x[0],y)

                        else:
                            x = (1/m)*next_x[1] + s_p[0] - (1/m)* s_p[1]
                            next_p = (x,next_x[1])

                        path_segment = PathSegment(
                        start_point=s_p, 
                        end_point=next_p, 
                        segment_type='line',
                        )

                        path_segments.append(path_segment)

                        s_p = next_p
                    
                    path_segment = PathSegment(
                        start_point=s_p, 
                        end_point=(x_eps_left, y_eps_left), 
                        segment_type='line',
                        )

                    path_segments.append(path_segment)

                else:
                    path_segment = PathSegment(
                        start_point=(x_eps_right_prev, y_eps_right_prev), 
                        end_point=(x_eps_left, y_eps_left),
                        segment_type='line', 
                    )

                    path_segments.append(path_segment)

            # Create the parabola segment between left epsilon point and right epsilon point
            path_segment = PathSegment(
                start_point=(x_eps_left, y_eps_left), 
                end_point=(x_eps_right, y_eps_right), 
                segment_type='parabola', 
            )

            path_segments.append(path_segment)

            # Special case: For the last waypoint, ensure the segment from the last right epsilon point to the final waypoint is a straight line
            if i == len(processed_waypoints) - 2:

                final_waypoint = processed_waypoints[-1]


                if processed_indices[i+1] - processed_indices[i] > n:

                    s_p = (x_eps_right, y_eps_right)
                    if (final_waypoint[0]-s_p[0]) == 0:
                        m = 10**6
                    else:
                        m = (final_waypoint[1]-s_p[1])/(final_waypoint[0]-s_p[0])

                    while processed_indices[i+1] - processed_indices[i] - n*k > n/2:
                        k+=1

                        next_x = waypoints[processed_indices[i-1]+n*k]

                        if np.abs(m)<50:
                            y = m* next_x[0] + s_p[1]- m*s_p[0]
                            next_p = (next_x[0],y)

                        else:
                            x = (1/m)*next_x[1] + s_p[0] - (1/m)* s_p[1]
                            next_p = (x,next_x[1])

                        path_segment = PathSegment(
                        start_point=s_p, 
                        end_point=next_p, 
                        segment_type='line',
                        )

                        path_segments.append(path_segment)

                        s_p = next_p
                    
                    path_segment = PathSegment(
                        start_point=s_p, 
                        end_point=final_waypoint, 
                        segment_type='line',
                        )

                    path_segments.append(path_segment)

                else:
                    path_segment = PathSegment(
                        start_point=(x_eps_right, y_eps_right), 
                        end_point=final_waypoint,
                        segment_type='line', 
                    )

                    path_segments.append(path_segment)

            # Update the previous right epsilon point and its time for the next iteration
            x_eps_right_prev, y_eps_right_prev = x_eps_right, y_eps_right

    prev_end_time=None

    for i, segment in enumerate(path_segments):

        if segment.segment_type == 'line':

            if prev_end_time==None:
                segment.straight_line_tf(0,v_max,epsilon)        
            else:
                segment.straight_line_tf(prev_end_time,v_max,epsilon)
                
        if segment.segment_type == 'parabola':
            # Get the endpoints of the parabola
            p1 = segment.start_point
            p2 = segment.end_point

            # Use the first coefficient of the previous and next segment for gradients
            if i > 0:  # Ensure we have a previous segment
                prev_segment = path_segments[i - 1]
                grad_left = find_grad(prev_segment)  # The first coefficient is the gradient
            else:
                grad_left = 0  # Set to zero if it's the first segment (or some default value)

            if i < len(path_segments) - 1:  # Ensure we have a next segment
                next_segment = path_segments[i + 1]
                grad_right = find_grad(next_segment)  # The first coefficient is the gradient
            else:
                grad_right = 0  # Set to zero if it's the last segment (or some default value)

            # Call the fit_parabola_to_epsilon_points function with the inputs
            coefficients = fit_parabola_to_epsilon_points(p1[0], p1[1], p2[0], p2[1], grad_left, grad_right)

            a = coefficients['a']
            b = coefficients['b']
            c = coefficients['c']
            d = coefficients['d']
            e = coefficients['e']
            f = coefficients['f']

            # Calculate the rotation angle
            theta = rotation_angle(a, b, c)

            # Rotate the parabola coefficients
            A_prime, B_prime, C_prime, D_prime, E_prime, F_prime = rotate_parabola(a, b, c, d, e, f, theta)

            segment.parab_tf(prev_end_time,theta,v_max,A_prime, B_prime, C_prime, D_prime, E_prime, F_prime,epsilon)        


        prev_end_time = segment.end_time

    return path_segments
    
def f(segments, s):
    result = ca.MX.zeros(3)  # Initialize a CasADi variable to store the selected (x, y) result

    for segment in segments:
        # Use ca.logic_and to check if s falls within the current segment's time bounds
        condition = ca.logic_and(s >= segment.start_time, s <= segment.end_time)
        
        # Use ca.if_else to choose the correct segment
        result = ca.if_else(condition,
                            segment.f(s),  # Use this segment if the condition is true
                            result)        # Keep the current result otherwise
    
    # Handle the case where s exceeds the last segment's end_time
    last_segment = segments[-1]
    result = ca.if_else(s > last_segment.end_time,
                        last_segment.f(last_segment.end_time),  # Return the last segment's endpoint
                        result)
    
    return result


def inf_path(s):

    T = 90

    etat1 = 5.7 * np.cos((2 * np.pi / T) * s)

    etat2 = 3.8 * np.sin((4 * np.pi / T) * s)

    eta = (etat1, etat2, 0)

    return eta


def generate_path_and_save(filename="path_points.csv"):

    path_points = []
    n= 1000
    for i in range(n):

        path_points.append(inf_path((i/n)*90))

    # Save path points to CSV

    with open(filename, mode='w', newline='') as file:

        writer = csv.writer(file)

        writer.writerow(['x', 'y'])

        for point in path_points:

            writer.writerow(point[:2])  # Only save x and y coordinates
    return path_points

points = np.array(generate_path_and_save())

# path_segments = LSPB_fit(55,0.6)
path_segments = LSPB_fit(20,0.15,0.1)

# Define the CasADi variable for s
s = ca.MX.sym('s')

# Select the correct segment and calculate (x, y) for the given value of s
selected_result = f(path_segments, s)

# Define a CasADi function to evaluate the selected (x, y) for a given s
f_s = ca.Function('f_s', [s], [selected_result])

# T_z_with_random_s_per_segment_with_transform_plot(path_segments, f_s,v_max=0.1,eps=0.15)


# fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

# circle = plt.Circle((4, 3.5), 0.5, color='blue', fill=False)
# ax1.plot(points[:,0], points[:,1], 'r-', label="Full Path using f_s")

# ax1.add_patch(circle)


# for i, segment in enumerate(path_segments):

#     if i>=0:
#         if segment.segment_type == 'line':
#             s = np.linspace(segment.start_time, segment.end_time-0.01, 1000)
#             x_vals = []
#             y_vals = []

#             for s_value in s:
#                 result = f_s(s_value)
#                 x_vals.append(float(result[0]))
#                 y_vals.append(float(result[1]))

#             ax1.plot(x_vals, y_vals, 'r-', label="Full Path using f_s")
#             ax1.set_title(f"Original Segment and Input Positions")
#             ax1.set_xlabel('x')
#             ax1.set_ylabel('y')
#             ax1.grid(True)

#             ax1.plot(segment.start_point[0],segment.start_point[1],'ko')
#             ax1.plot(segment.end_point[0],segment.end_point[1],'ko')

#             tfx = []
#             tfy = []

#             for i in range(len(x_vals)):
#                 point = segment.inv_transform_p((x_vals[i],y_vals[i]))
#                 tfx.append(point[0])
#                 tfy.append(point[1])

#             ax2.plot(tfx,tfy,'g-')


#         if segment.segment_type == 'parabola':
            
#             s = np.linspace(segment.start_time, segment.end_time-0.01, 1000)
#             x_vals = []
#             y_vals = []

#             for s_value in s:
#                 result = f_s(s_value)
#                 x_vals.append(float(result[0]))
#                 y_vals.append(float(result[1]))

#             ax1.plot(x_vals, y_vals, 'b-', label="Full Path using f_s")
#             ax1.set_title(f"Original Segment and Input Positions")
#             ax1.set_xlabel('x')
#             ax1.set_ylabel('y')
#             ax1.grid(True)

#             tfx = []
#             tfy = []

#             for i in range(len(x_vals)):
#                 point = segment.inv_transform_p((x_vals[i],y_vals[i]))
#                 tfx.append(point[0])
#                 tfy.append(point[1])

#             ax2.plot(tfx,tfy,'g-')

#             # print(segment.eta)

# plt.show()


class NMPCController:
    def __init__(self):
        # NMPC Parameters
        self.Ts = 0.3  # Sampling time
        self.Ts_sim = 0.05  # Smaller simulation step time
        self.N = 60  # Prediction horizon
        self.nx = 4  # State dimension (x, y, theta, s)
        self.nu = 3  # Input dimension (v, omega, w)

        # Initialize `s0` and get the initial state from the first point of `f_s`
        self.s0 = 0
        self.initial_point = f_s(0).full().flatten()  # Store the initial point for reuse
        self.current_state = np.array([self.initial_point[0], self.initial_point[1], 0, self.s0])  # Assume theta = 0

        # Initialize control variables
        self.u0 = np.array([1, 0])
        self.w0 = 1

        # Set obstacle at (3, 0.5) with a radius of 0.03
        self.max_obs = 1

        self.obs_s = 35
        self.obs_d = 0.58

        self.obs_x, self.obs_y = get_deviated_point(path_segments, self.obs_s, self.obs_d)

        self.obs_list = np.array([[self.obs_x, self.obs_y, 0.17]])  # Single obstacle

        # Setup MPC
        self.ocp = self.setup_ocp_with_cost_function()
        self.solver = AcadosOcpSolver(self.ocp, json_file="acados_ocp.json")

        self.closed_loop_trajectory = []

        self.final_position = f_s(path_segments[-1].end_time).full().flatten()


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
        obs_list = ca.SX.sym("obs", self.max_obs, 3)

        dx = v * ca.cos(theta)
        dy = v * ca.sin(theta)
        dtheta = omega
        ds = w

        xdot = ca.vertcat(dx, dy, dtheta, ds)

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

        N = self.N
        Ts = self.Ts
        T = N * Ts
        mu = 8 * 10**2

        ocp.dims.N = N
        ocp.solver_options.tf = T

        Q = np.diag([10, 10, 0])  # State weights (3x3)
        R = np.diag([1, 1])     # Control input weights (2x2)
        T_cost = np.array([[10]])     # Weight for time dilation cost (1x1)

        x = ocp.model.x[:3]  # State vector [x, y, theta]
        u = ocp.model.u[:2]  # Control vector [v, omega]
        w = ocp.model.u[2]   # Third control input (scalar)
        s = ocp.model.x[3]   # Path parameter

        obs_list = ca.reshape(ocp.model.p, self.max_obs, 3)

        # Reference trajectory using f_s
        xi_s = f_s(s)
        dx = x - xi_s

        # Cost function expressions
        ocp.model.cost_y_expr = ca.vertcat(dx, u, (1 - w))
        for i in range(self.max_obs):
            h = ca.if_else(
                0.1 > 0,
                ca.fmax((0.17+0.23+0.18)**2 - (x[0] - self.obs_x)**2 - (x[1] - self.obs_y)**2, 0),
                0
            )
            ocp.model.cost_y_expr = ca.vertcat(ocp.model.cost_y_expr, h)

        ocp.model.cost_y_expr_e = ca.vertcat(dx)

        # Create the weight matrix with correct dimensions
        num_obstacle_terms = self.max_obs
        obstacle_weights = 0.5 * mu * np.eye(num_obstacle_terms)

        ocp.cost.W = np.block([
            [Q, np.zeros((3, 2)), np.zeros((3, 1)), np.zeros((3, num_obstacle_terms))],
            [np.zeros((2, 3)), R, np.zeros((2, 1)), np.zeros((2, num_obstacle_terms))],
            [np.zeros((1, 3)), np.zeros((1, 2)), T_cost, np.zeros((1, num_obstacle_terms))],
            [np.zeros((num_obstacle_terms, 3)), np.zeros((num_obstacle_terms, 2)), np.zeros((num_obstacle_terms, 1)), obstacle_weights]
        ])

        ocp.cost.W_e = Q

        ny = ocp.model.cost_y_expr.size()[0]
        ocp.cost.yref = np.zeros((ny,))
        ocp.cost.yref_e = np.zeros((dx.size()[0],))

        ocp.constraints.lbx = np.array([0])  # Lower bound for `s`
        ocp.constraints.ubx = np.array([path_segments[-1].end_time])  # Upper bound for `s`
        ocp.constraints.idxbx = np.array([3])  # Constraining the `s` state only

        ocp.parameter_values = np.zeros((self.max_obs * 3, 1))

        ocp.constraints.lbu = np.array([0, -0.8, 0])
        ocp.constraints.ubu = np.array([1, 0.8, 1])
        ocp.constraints.idxbu = np.array([0, 1, 2])

        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"

        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.tf = T
        ocp.solver_options.nlp_solver_max_iter = 500
        ocp.solver_options.qp_solver_iter_max = 500
        ocp.solver_options.print_level = 0
        ocp.solver_options.regularize_method = "MIRROR"
        ocp.solver_options.levenberg_marquardt = 1e-4

        # Use the stored initial_point for the initial state
        x0_initial = np.array([self.initial_point[0], self.initial_point[1], 0, self.s0])  # Initial state with s0
        ocp.constraints.x0 = x0_initial

        return ocp


    def run_mpc(self):
        self.solver.set(0, "p", self.obs_list.flatten())
        self.solver.set(0, "lbx", self.current_state)
        self.solver.set(0, "ubx", self.current_state)

        status = self.solver.solve()

        if status != 0:
            print(f"ACADOS returned status {status}")

        usol = self.solver.get(0, "u")
        x_opt = [self.solver.get(i, "x") for i in range(self.N + 1)]
        return usol, x_opt


    def plot_results(self, x_opt):
        # Plot the reference path
        s_values = np.linspace(0, path_segments[-1].end_time, 500)
        ref_path = np.array([f_s(s).full().flatten() for s in s_values])
        
        plt.figure(figsize=(10, 10))
        
        # Plot reference path
        plt.plot(ref_path[:, 0], ref_path[:, 1], 'g--', label='Reference Path')

        # Plot obstacles
        for obs in self.obs_list:
            circle = plt.Circle((obs[0], obs[1]), obs[2], color='red', alpha=0.5)
            plt.gca().add_patch(circle)

        # Plot x_opt trajectory
        x_opt = np.array(x_opt)
        plt.plot(x_opt[:, 0], x_opt[:, 1], 'b-', label='Optimal Trajectory')

        # Plot settings
        plt.scatter(self.initial_point[0], self.initial_point[1], color='black', label='Start Point')
        plt.title('Reference Path, Obstacles, and Optimal Trajectory')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

    def simulate_kinematic_step(self, state, control, Ts):
        x, y, theta, s = state
        v, omega = control[:2]
        x_next = x + Ts * v * np.cos(theta)
        y_next = y + Ts * v * np.sin(theta)
        theta_next = theta + Ts * omega
        s_next = s + Ts * control[2]  # Increment s using the third control input (w)
        return np.array([x_next, y_next, theta_next, s_next])
    
    def run_mpc_loop(self):
        # Run the loop until the current position is within 0.1 units of the final position
        while np.linalg.norm(self.current_state[:2] - self.final_position[:2]) > 0.1:
            self.solver.set(0, "p", self.obs_list.flatten())
            self.solver.set(0, "lbx", self.current_state)
            self.solver.set(0, "ubx", self.current_state)

            status = self.solver.solve()

            if status != 0:
                print(f"ACADOS returned status {status}")
                break

            usol = self.solver.get(0, "u")
            x_opt = [self.solver.get(i, "x") for i in range(self.N + 1)]

            # Apply the first control input to the kinematic model
            self.current_state = self.simulate_kinematic_step(self.current_state, usol, self.Ts_sim)

            # Update s0 based on the third control input from usol
            self.s0 = self.current_state[3]

            # Store the current state for plotting
            self.closed_loop_trajectory.append(self.current_state[:3])  # Store (x, y, theta)

    def run_mpc_loop_n(self, n=50):
        plt.figure(figsize=(10, 10))

        for _ in range(n):
            self.solver.set(0, "p", self.obs_list.flatten())
            self.solver.set(0, "lbx", self.current_state)
            self.solver.set(0, "ubx", self.current_state)

            status = self.solver.solve()

            if status != 0:
                print(f"ACADOS returned status {status}")
                break

            usol = self.solver.get(0, "u")
            x_opt = np.array([self.solver.get(i, "x") for i in range(self.N + 1)])

            # plt.plot(x_opt[:, 0], x_opt[:, 1], 'r--')

            # Apply the first control input to the kinematic model
            self.current_state = self.simulate_kinematic_step(self.current_state, usol, self.Ts_sim)

            # Update s0 based on the third control input from usol
            self.s0 = self.current_state[3]

            # Store the current state for plotting
            self.closed_loop_trajectory.append(self.current_state[:3])  # Store (x, y, theta)

    def plot_closed_loop(self):
        self.closed_loop_trajectory = np.array(self.closed_loop_trajectory)
        s_values = np.linspace(0, path_segments[-1].end_time, 500)
        ref_path = np.array([f_s(s).full().flatten() for s in s_values])

        
        plt.plot(ref_path[:, 0], ref_path[:, 1], 'g--', label='Reference Path')

        for obs in self.obs_list:
            circle = plt.Circle((obs[0], obs[1]), obs[2], color='red', alpha=0.5)
            plt.gca().add_patch(circle)

        # for i in range(len(self.closed_loop_trajectory)):
        #     circle = plt.Circle((self.closed_loop_trajectory[i, 0], self.closed_loop_trajectory[i, 1]), 0.23/2, color='green', alpha=0.5)
        #     plt.gca().add_patch(circle)

        plt.plot(self.closed_loop_trajectory[:, 0], self.closed_loop_trajectory[:, 1], 'b-', label='Closed-Loop Trajectory')
        plt.scatter(self.initial_point[0], self.initial_point[1], color='black', label='Start Point')
        plt.title('Closed-Loop Trajectory, Reference Path, and Obstacles')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

if __name__ == "__main__":
    controller = NMPCController()
    # usol, x_opt = controller.run_mpc()
    # print("Optimal control input:", usol)
    # print("Optimal state trajectory:", x_opt)

    # # Plot results
    # controller.plot_results(x_opt)

    controller.run_mpc_loop_n(n=1400)
    controller.plot_closed_loop()