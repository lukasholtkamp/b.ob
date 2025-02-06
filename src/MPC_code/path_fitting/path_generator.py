
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
import pandas as pd
import casadi as ca
import random
import control as ct
import csv
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import json

from matplotlib.patches import Circle

from waypoint_filter import *
from line_fitting import *
from transform import *
from path_segments_save import *
from plotting import *

<<<<<<< HEAD
# # Configure PGF for LaTeX export
# rcParams.update({
#     "pgf.texsystem": "pdflatex",  # Use pdflatex or xelatex
#     "text.usetex": True,          # Enable LaTeX text rendering
#     "font.family": "serif",       # Match LaTeX document fonts
#     "pgf.preamble": [
#         r"\usepackage{amsmath}",  # Use additional LaTeX packages if needed
#     ]
# })
=======
# Configure PGF for LaTeX export
rcParams.update({
    "pgf.texsystem": "pdflatex",  # Use pdflatex or xelatex
    "text.usetex": True,          # Enable LaTeX text rendering
    "font.family": "serif",       # Match LaTeX document fonts
    "pgf.preamble": [
        r"\usepackage{amsmath}",  # Use additional LaTeX packages if needed
    ],
    'font.size': 18
})
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

def LSPB_fit(n,epsilon,v_max=0.1):

    # file_path = '/home/bertrandt/b.ob/src/MPC_code/path_fitting/path_data_log_right.csv'
    # file_path = '/home/bertrandt/b.ob/src/MPC_code/path_fitting/replan_sim/plan_right_xy.csv'
    file_path = '/home/bertrandt/b.ob/src/MPC_code/path_fitting/replan_real/plan_xy.csv'



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

def save_path_segments(path_segments, filename="path_segments_1.json"):
    """Save a list of PathSegment objects to a JSON file."""
    segments_data = [segment.to_dict() for segment in path_segments]
    with open(filename, "w") as f:
        json.dump(segments_data, f, indent=4)
    print(f"Path segments saved to {filename}")

def load_path_segments(filename="path_segments_1.json"):
    """Load a list of PathSegment objects from a JSON file."""
    with open(filename, "r") as f:
        segments_data = json.load(f)
    path_segments = [PathSegment.from_dict(segment) for segment in segments_data]
    print(f"Path segments loaded from {filename}")
    return path_segments


points = np.array(generate_path_and_save())

# path_segments = LSPB_fit(55,0.6)
path_segments_og = LSPB_fit(20,0.15,0.1)

save_path_segments(path_segments_og)

path_segments = load_path_segments()

# Define the CasADi variable for s
s = ca.MX.sym('s')

# Select the correct segment and calculate (x, y) for the given value of s
selected_result = f(path_segments, s)

# Define a CasADi function to evaluate the selected (x, y) for a given s
f_s = ca.Function('f_s', [s], [selected_result])

# T_z_with_random_s_per_segment_with_transform_plot(path_segments, f_s,v_max=0.1,eps=0.15)

# from matplotlib.lines import Line2D
<<<<<<< HEAD

# fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

# # circle = plt.Circle((4, 3.5), 0.5, color='blue', fill=False)
# # ax1.plot(points[:,0], points[:,1], 'r-', label="Full Path using f_s")

# # ax1.add_patch(circle)

# file_path = '/home/bertrandt/b.ob/src/MPC_code/path_fitting/path_data_log_2.csv'

# # Get the selected waypoints and their original indices
# original_indices, selected_waypoints, waypoints = select_waypoints_with_indices(file_path, 20)

# ax1.plot(waypoints[:, 0], waypoints[:, 1], color='orange', alpha=0.7, linewidth=2)
=======

# fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6), constrained_layout=True)

# # circle = plt.Circle((4, 3.5), 0.5, color='blue', fill=False)
# # ax1.plot(points[:,0], points[:,1], 'r-', label="Full Path using f_s")
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

# # ax1.add_patch(circle)

# file_path = '/home/bertrandt/b.ob/src/MPC_code/path_fitting/path_data_log_right.csv'

# # Get the selected waypoints and their original indices
# original_indices, selected_waypoints, waypoints = select_waypoints_with_indices(file_path, 20)

# ax1.plot(waypoints[:, 0], waypoints[:, 1], color='orange', alpha=0.7, linewidth=2)


# etas = []

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

#             # ax1.plot(segment.start_point[0],segment.start_point[1],'ko')
#             # ax1.plot(segment.end_point[0],segment.end_point[1],'ko')

#             tfx = []
#             tfy = []

#             for i in range(len(x_vals)):
#                 point = segment.inv_transform_p((x_vals[i],y_vals[i]))
#                 tfx.append(point[0])
#                 tfy.append(point[1])

#             ax2.plot(tfx,tfy,'r-')
<<<<<<< HEAD
=======

#             etas.append(segment.eta)

>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097


#         if segment.segment_type == 'parabola':
            
#             s = np.linspace(segment.start_time, segment.end_time-0.01, 1000)
#             x_vals = []
#             y_vals = []

#             for s_value in s:
#                 result = f_s(s_value)
#                 x_vals.append(float(result[0]))
#                 y_vals.append(float(result[1]))

#             ax1.plot(x_vals, y_vals, 'b-', label="Full Path using f_s")

#             tfx = []
#             tfy = []

#             for i in range(len(x_vals)):
#                 point = segment.inv_transform_p((x_vals[i],y_vals[i]))
#                 tfx.append(point[0])
#                 tfy.append(point[1])

#             ax2.plot(tfx,tfy,'b-')

#             # print(segment.eta)
#             etas.append(segment.eta)


<<<<<<< HEAD
# ax1.set_xlabel('x (m)')
# ax1.set_ylabel('y (m)')
=======
# ax1.set_xlabel('x [m]')
# ax1.set_ylabel('y [m]')
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097
# ax1.grid(True)

# # Create custom legend entries
# custom_lines = [
#     Line2D([0], [0], color='orange', alpha=0.7, linewidth=2, linestyle='-'),  # Original waypoints
#     Line2D([0], [0], color='red', linewidth=2, linestyle='-'),             # Straight line segments
#     Line2D([0], [0], color='blue', linewidth=2, linestyle='-'),            # Parabolic segments
# ]

# # Add the legend with custom labels
# ax1.legend(
#     custom_lines,
#     ['Original Waypoints', 'Straight Line Segments', 'Parabolic Segments'],
<<<<<<< HEAD
#     loc='upper left',  # Position the legend
#     fontsize=10,        # Set the font size
=======
#     loc='lower left',  # Position the legend
#     fontsize=1,        # Set the font size
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097
#     frameon=True,       # Add a box around the legend
#     framealpha=0.8,     # Adjust transparency of the legend box
#     edgecolor='black'   # Set the border color of the legend box
# )

<<<<<<< HEAD
# ax2.set_xlabel('\^x (m)')
# ax2.set_ylabel('\^y (m)')
=======
# ax2.set_xlabel('\^x [m]')
# ax2.set_ylabel('\^y [m]')
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097
# ax2.grid(True)

# # Create custom legend entries
# custom_lines = [
#     Line2D([0], [0], color='red', linewidth=2, linestyle='-'),             # Straight line segments
#     Line2D([0], [0], color='blue', linewidth=2, linestyle='-'),            # Parabolic segments
# ]

# # Add the legend with custom labels
# ax2.legend(
#     custom_lines,
#     ['Transformed Straight Line Segments', 'Transformed Parabolic Segments'],
<<<<<<< HEAD
#     loc='upper left',  # Position the legend
#     fontsize=10,        # Set the font size
=======
#     loc='lower left',  # Position the legend
#     fontsize=14,        # Set the font size
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097
#     frameon=True,       # Add a box around the legend
#     framealpha=0.8,     # Adjust transparency of the legend box
#     edgecolor='black'   # Set the border color of the legend box
# )


<<<<<<< HEAD


# # Save as PGF file
# plt.savefig("plots/path_plot.pgf", bbox_inches='tight')  # Export to PGF for LaTeX integration
=======
# # plt.figure()

# # # Create a histogram
# # plt.hist(etas, bins=10, edgecolor='black', alpha=0.7)

# # # Add labels and title
# # plt.xlabel(r'$\eta$')
# # plt.ylabel('Frequency')

# # Save as PGF file
# plt.savefig("plots/path_plot_2.pgf",bbox_inches='tight')  # Export to PGF for LaTeX integration
# # plt.savefig("plots/etas_2.pgf", bbox_inches='tight')  # Export to PGF for LaTeX integration

>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

# # plt.show()


from tensorflow.keras.models import load_model
import tensorflow as tf
import keras
# Enable unsafe deserialization
keras.config.enable_unsafe_deserialization()

import torch


import time
from tabulate import tabulate
from policy_model import PolicyModel


class CollisionPenaltyLayer(tf.keras.layers.Layer):
    def __init__(self, safety_margin=0.41, penalty_weight=10, **kwargs):
        super(CollisionPenaltyLayer, self).__init__(**kwargs)
        self.safety_margin = safety_margin
        self.penalty_weight = penalty_weight

    def call(self, inputs):
        # Extract input components
        x, y = inputs[:, 0], inputs[:, 1]
        obs_x, obs_y, obs_r = inputs[:, 5], inputs[:, 6], inputs[:, 7]

        # Compute distance to the obstacle
        dist_to_obs = tf.sqrt((x - obs_x)**2 + (y - obs_y)**2)

        # Compute collision penalty
        collision_penalty = tf.nn.relu(self.safety_margin - dist_to_obs)

        # Scale the penalty
        scaled_penalty = self.penalty_weight * collision_penalty
        return scaled_penalty

class NMPCController:
    def __init__(self, r1=0.15,r2=0.17,r3=0.2,infl=0.3):
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

<<<<<<< HEAD
        self.obs_inflation = 0.22
        self.obs_list = [
            {"s": 15, "d": 0.2, "r": 0.15 + self.obs_inflation},
            {"s": 23, "d": 0.1, "r": 0.17 + self.obs_inflation},
            {"s": 30, "d": -0.2, "r": 0.2 + self.obs_inflation}
        ]

        # self.obs_inflation = 0
        # self.obs_list = [
        #     {"s": 55, "d": 0.01, "r": 0},
        #     {"s": 15, "d": 0.1, "r": 0},
        #     {"s": 32, "d": -0.2, "r": 0}
        # ]

=======
        self.obs_inflation = infl
        self.obs_list = [
            {"s": 15, "d": -0.23, "r": r1 + self.obs_inflation},
            {"s": 30, "d": -0.3, "r": r3 + self.obs_inflation},
        ]

        # self.obs_inflation = infl
        # self.obs_list = [
        #     {"s": 20, "d": 0.1, "r": 0.12 + self.obs_inflation}
        # ]

        # self.obs_inflation = 0
        # self.obs_list = [
        #     {"s": 55, "d": 0.01, "r": 0},
        #     {"s": 15, "d": 0.1, "r": 0},
        #     {"s": 32, "d": -0.2, "r": 0}
        # ]

>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097
        # Compute obstacle positions and add to the list
        for obs in self.obs_list:
            obs["x"], obs["y"] = get_deviated_point(path_segments, obs["s"], obs["d"])

        # Setup MPC
        self.ocp = self.setup_ocp_with_cost_function()
        self.solver = AcadosOcpSolver(self.ocp, json_file="acados_ocp.json")

        self.closed_loop_trajectory = []
        self.final_position = f_s(path_segments[-1].end_time).full().flatten()

        self.obs_model = load_model("/home/bertrandt/b.ob/src/MPC_code/IL/path_following_obs_avoidance_attention.h5")

        # self.obs_model = tf.keras.models.load_model("/home/bertrandt/b.ob/src/MPC_code/IL/path_following_obs_avoidance_with_penalty.keras",custom_objects={"CollisionPenaltyLayer": CollisionPenaltyLayer})
        
        self.pf_model = PolicyModel(input_dim=5, output_dim=3)  # Replace with your actual architecture
        self.pf_model.load_state_dict(torch.load("/home/bertrandt/b.ob/src/MPC_code/IL/DPL/models/final_policy.pth"))
        self.pf_model.eval()  # Set the model to evaluation mode

<<<<<<< HEAD
=======
        self.plot_initialized = False
        self.ref_plotted = False
        self.state_ref_plotted = False


    def initialize_plots(self):
        """Initialize separate figures for overlaying results."""
        if not self.plot_initialized:
            # Create figure for trajectories
            self.fig_traj = plt.figure(figsize=(10, 8))
            self.ax_traj = self.fig_traj.add_subplot(1, 1, 1)
            # self.ax_traj.set_title("Trajectories")
            self.ax_traj.set_xlabel("x [m]")
            self.ax_traj.set_ylabel("y [m]")
            self.ax_traj.grid()

            # Create figure for Cartesian error
            self.fig_error = plt.figure(figsize=(10, 8))
            self.ax_error = self.fig_error.add_subplot(1, 1, 1)
            # self.ax_error.set_title("Cartesian Error vs. s0")
            self.ax_error.set_xlabel(r"$\theta$")
            self.ax_error.set_ylabel(r"log$_{10}$(Cartesian Error) [m]")

            # Create figure for X state
            self.fig_states = plt.figure(figsize=(10, 8))
            self.ax_python_states_x = self.fig_states.add_subplot(4, 1, 1)
            self.ax_python_states_x.set_ylabel("x [m]")

            self.ax_python_states_y = self.fig_states.add_subplot(4, 1, 2)
            self.ax_python_states_y.set_ylabel("y [m]")
            self.ax_python_states_y.set_xlabel(r"$\theta$")

            self.ax_gazebo_states_x = self.fig_states.add_subplot(4, 1, 3)
            self.ax_gazebo_states_x.set_ylabel("x [m]")

            self.ax_gazebo_states_y = self.fig_states.add_subplot(4, 1, 4)
            self.ax_gazebo_states_y.set_ylabel("y [m]")
            self.ax_gazebo_states_y.set_xlabel(r"$\theta$")

            # Control Inputs
            self.fig_control = plt.figure(figsize=(10, 8))
            self.ax_python_control = self.fig_control.add_subplot(2, 1, 1)
            self.ax_python_control.set_ylabel("Control Input Values")
            self.ax_python_control.set_xlabel(r"$\theta$")

            self.ax_gazebo_control = self.fig_control.add_subplot(2, 1, 2)
            self.ax_gazebo_control.set_ylabel("Control Input Values")
            self.ax_gazebo_control.set_xlabel(r"$\theta$")

            
            self.fig_control_single = plt.figure(figsize=(10, 8))
            self.ax_control = self.fig_control_single.add_subplot(1, 1, 1)
            self.ax_control.set_ylabel("Control Input Values")
            self.ax_control.set_xlabel(r"$\theta$")


            self.plot_initialized = True

>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

    def predict_policy(self, input_data):
        # Convert the input data to a PyTorch tensor
        input_tensor = torch.tensor(input_data, dtype=torch.float32)
        # Perform the prediction
        with torch.no_grad():
            output = self.pf_model(input_tensor)
        return output.numpy()  # Convert output to NumPy array for further processing
    
    def find_closest_obstacle(self):
        """Find the closest obstacle to the current robot position."""
        current_x, current_y = self.current_state[0], self.current_state[1]
        closest_obs = min(
            self.obs_list,
            key=lambda obs: np.sqrt((current_x - obs["x"])**2 + (current_y - obs["y"])**2)
        )
        return np.array([closest_obs["x"], closest_obs["y"], closest_obs["r"]])


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

        N = self.N
        Ts = self.Ts
        T = N * Ts
        mu = 8 * 10**2

        ocp.dims.N = N
        ocp.solver_options.tf = T

        Q = np.diag([10, 10, 0])  # State weights (3x3)
        R = np.diag([1, 1])       # Control input weights (2x2)
        T_cost = np.array([[10]]) # Weight for time dilation cost (1x1)

        x = ocp.model.x[:3]  # State vector [x, y, theta]
        u = ocp.model.u[:2]  # Control vector [v, omega]
        w = ocp.model.u[2]   # Third control input (scalar)
        s = ocp.model.x[3]   # Path parameter

        obs_list = ca.reshape(ocp.model.p, 1, 3)  # Single obstacle as parameter

        # Reference trajectory using f_s
        xi_s = f_s(s)
        dx = x - xi_s

        # Cost function expressions
        ocp.model.cost_y_expr = ca.vertcat(dx, u, (1 - w))
        obs_x = obs_list[0, 0]
        obs_y = obs_list[0, 1]
        obs_r = obs_list[0, 2]

        # Obstacle avoidance penalty
        h = ca.if_else(obs_r > 0, ca.fmax(obs_r**2 - (x[0] - obs_x)**2 - (x[1] - obs_y)**2, 0), 0)
        ocp.model.cost_y_expr = ca.vertcat(ocp.model.cost_y_expr, h)

        ocp.model.cost_y_expr_e = ca.vertcat(dx)

        # Create the weight matrix with correct dimensions
        obstacle_weights = 0.5 * mu

        ocp.cost.W = np.block([
            [Q, np.zeros((3, 2)), np.zeros((3, 1)), np.zeros((3, 1))],
            [np.zeros((2, 3)), R, np.zeros((2, 1)), np.zeros((2, 1))],
            [np.zeros((1, 3)), np.zeros((1, 2)), T_cost, np.zeros((1, 1))],
            [np.zeros((1, 3)), np.zeros((1, 2)), np.zeros((1, 1)), obstacle_weights]
        ])

        ocp.cost.W_e = Q

        ny = ocp.model.cost_y_expr.size()[0]
        ocp.cost.yref = np.zeros((ny,))
        ocp.cost.yref_e = np.zeros((dx.size()[0],))

        ocp.constraints.lbx = np.array([0])  # Lower bound for `s`
        ocp.constraints.ubx = np.array([path_segments[-1].end_time])  # Upper bound for `s`
        ocp.constraints.idxbx = np.array([3])  # Constraining the `s` state only

        ocp.parameter_values = np.zeros((3,))  # Single obstacle parameter

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

        x0_initial = np.array([self.initial_point[0], self.initial_point[1], 0, self.s0])
        ocp.constraints.x0 = x0_initial

        return ocp

    def simulate_kinematic_step(self, state, control, Ts):
        x, y, theta, s = state
        v, omega = control[:2]
        x_next = x + Ts * v * np.cos(theta)
        y_next = y + Ts * v * np.sin(theta)
        theta_next = theta + Ts * omega
        s_next = s + Ts * control[2]  # Increment s using the third control input (w)
        return np.array([x_next, y_next, theta_next, s_next])
<<<<<<< HEAD
    

    def run_mpc_loop(self):
        self.closed_loop_controls = []  # Store control inputs
        solver_times = []  # Store solver times for each iteration

=======

    def run_mpc_loop(self):
        closed_loop_controls = []  # Store control inputs
        closed_loop_trajectory = []
        solver_times = []  # Store solver times for each iteration


>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097
        while np.linalg.norm(self.current_state[:2] - self.final_position[:2]) > 0.1:
            # Find the closest obstacle
            closest_obs = self.find_closest_obstacle()

            # Set the closest obstacle as the parameter for all stages
            for k in range(self.N):
                self.solver.set(k, "p", closest_obs.flatten())

            self.solver.set(0, "lbx", self.current_state)
            self.solver.set(0, "ubx", self.current_state)

            # Measure time for solving the MPC
            start_time = time.time()
            status = self.solver.solve()
            end_time = time.time()
<<<<<<< HEAD
            solver_time = end_time - start_time
            solver_times.append(solver_time)
=======
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

            if status != 0:
                print(f"ACADOS returned status {status}. Exiting loop.")
                break  # Exit loop on solver failure

            # Store solver time
            solver_time = end_time - start_time
            solver_times.append(solver_time)

            usol = self.solver.get(0, "u")
            x_opt = [self.solver.get(i, "x") for i in range(self.N + 1)]

            # Apply the first control input to the kinematic model
            self.current_state = self.simulate_kinematic_step(self.current_state, usol, self.Ts_sim)
            self.s0 = self.current_state[3]

            # Store the full state (x, y, theta, s)
<<<<<<< HEAD
            self.closed_loop_trajectory.append(self.current_state.copy())

            # Store the control inputs
            self.closed_loop_controls.append(usol.copy())

        # Compute statistics for solver times
        mean_time = np.mean(solver_times)
        std_time = np.std(solver_times)
        worst_time = np.max(solver_times)

        # Display results in a table
        table = [["Metric", "Time (s)"],
                 ["Mean", f"{mean_time:.6f}"],
                 ["Standard Deviation", f"{std_time:.6f}"],
                 ["Worst Case", f"{worst_time:.6f}"]]
        print(tabulate(table, headers="firstrow", tablefmt="grid"))

=======
            closed_loop_trajectory.append(self.current_state.copy())
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

            # Store the control inputs
            closed_loop_controls.append(usol.copy())

        return closed_loop_trajectory,closed_loop_controls,solver_times

<<<<<<< HEAD
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

    def plot_closed_loop(self, csv_file=None):
        """
        Plot the closed-loop trajectory, reference path, and obstacles.
        Optionally overlay recorded data from a CSV file.
        """
        self.closed_loop_trajectory = np.array(self.closed_loop_trajectory)
        s_values = np.linspace(0, path_segments[-1].end_time, 500)
        ref_path = np.array([f_s(s).full().flatten() for s in s_values])

        plt.figure(figsize=(10, 10))

        # Plot the reference path
        plt.plot(ref_path[:, 0], ref_path[:, 1], 'g--', label='Reference Path')

        # Plot all obstacles from obs_list
        for obs in self.obs_list:
            obs_x = obs["x"]
            obs_y = obs["y"]
            obs_r = obs["r"] - self.obs_inflation
            circle = plt.Circle((obs_x, obs_y), obs_r, color='red', alpha=0.5)
            plt.gca().add_patch(circle)

        # Plot the closed-loop trajectory
        plt.plot(self.closed_loop_trajectory[:, 0], self.closed_loop_trajectory[:, 1], 'b-', label='Closed-Loop Trajectory')

        # Overlay CSV data if provided
        if csv_file:
            csv_data = self.load_csv_data(csv_file)
            plt.plot(csv_data[:, 0], csv_data[:, 1], 'r--', label='Recorded Data (CSV)')

        # Start point
        plt.scatter(self.initial_point[0], self.initial_point[1], color='black', label='Start Point')

        # Plot settings
        plt.title('Closed-Loop Trajectory, Reference Path, and Obstacles')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

        # Plot Cartesian error and inputs
        self.plot_cartesian_error_and_inputs(csv_file)


    def plot_cartesian_error_and_inputs(self, csv_file=None):
        """Plot the Cartesian error and control inputs against `s0`. Optionally overlay data from a CSV file."""
        # Extract s0 and positions from the stored trajectory
        s_history = [state[3] for state in self.closed_loop_trajectory]
        x_history = [state[0] for state in self.closed_loop_trajectory]
        y_history = [state[1] for state in self.closed_loop_trajectory]

        # Cartesian error computation
        cartesian_errors = []
        for s, x, y in zip(s_history, x_history, y_history):
            ref_pos = f_s(s).full().flatten()
            cartesian_error = np.sqrt((x - ref_pos[0])**2 + (y - ref_pos[1])**2)
            cartesian_errors.append(cartesian_error)

        # Control inputs
        v_inputs = [control[0] for control in self.closed_loop_controls]
        omega_inputs = [control[1] for control in self.closed_loop_controls]
        w_inputs = [control[2] for control in self.closed_loop_controls]

        # Plot Cartesian error vs. s0
        plt.figure(figsize=(10, 6))
        plt.plot(s_history, cartesian_errors, label="Cartesian Error (Simulated)", color="blue")

        # Overlay CSV data if provided
        if csv_file:
            csv_data = self.load_csv_data(csv_file)
            s_csv = csv_data[:, 3]  # Assuming `s0` is the fourth column
            x_csv = csv_data[:, 0]  # Assuming `x` is the first column
            y_csv = csv_data[:, 1]  # Assuming `y` is the second column
            csv_errors = [
                np.sqrt((x_csv[i] - f_s(s_csv[i]).full().flatten()[0])**2 +
                        (y_csv[i] - f_s(s_csv[i]).full().flatten()[1])**2)
                for i in range(len(s_csv))
            ]
            plt.plot(s_csv, csv_errors, label="Cartesian Error (CSV)", color="red", linestyle="--")

        plt.title("Cartesian Error vs. s0")
        plt.xlabel("s0 (Path Progress Parameter)")
        plt.ylabel("Cartesian Error (m)")
        plt.legend()
        plt.grid(True)
        plt.show()

        # Plot control inputs vs. s0
        plt.figure(figsize=(10, 6))
        plt.plot(s_history, v_inputs, label="Linear Velocity (v)", color="green")
        plt.plot(s_history, omega_inputs, label="Angular Velocity (ω)", color="red")
        plt.plot(s_history, w_inputs, label="Path Progress Rate (w)", color="orange")

        if csv_file:
            v_csv = csv_data[:, 4]  # Assuming `v` is the fifth column
            omega_csv = csv_data[:, 5]  # Assuming `omega` is the sixth column
            w_csv = csv_data[:, 6]  # Assuming `w` is the seventh column
            plt.plot(s_csv, v_csv, label="v (CSV)", color="green", linestyle="--")
            plt.plot(s_csv, omega_csv, label="ω (CSV)", color="red", linestyle="--")
            plt.plot(s_csv, w_csv, label="w (CSV)", color="orange", linestyle="--")

        plt.title("Control Inputs vs. s0")
        plt.xlabel("s0 (Path Progress Parameter)")
        plt.ylabel("Control Input Value")
        plt.legend()
        plt.grid(True)
        plt.show()


    def run_NN_obs_n(self,n=50):
=======
    def run_NN_obs(self,n=50):
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

        for _ in range(n):

            x_hat, y_hat, theta_hat, s_hat, eta,obs_x_hat,obs_y_hat,relevant_flag = T_z_obs(path_segments, self.current_state[0], self.current_state[1], self.current_state[2], self.current_state[3], self.obs_x, self.obs_y,self.obs_r)

            if eta>=0:
                if relevant_flag:
                    input = np.array([[x_hat, y_hat, theta_hat, s_hat, eta,obs_x_hat,obs_y_hat,self.obs_r+self.obs_inflation]])  # Shape: (1, 8)
                else:
                    input = np.array([[x_hat, y_hat, theta_hat, s_hat, eta,0,0,0]])  # Shape: (1, 8)
            else:
                if relevant_flag:
                    input = np.array([[x_hat, -y_hat, -theta_hat, s_hat, -eta,obs_x_hat,-obs_y_hat,self.obs_r+self.obs_inflation]])  # Shape: (1, 8)
                else:
                    input = np.array([[x_hat, -y_hat, -theta_hat, s_hat, -eta,0,0,0]])  # Shape: (1, 8)


            usol = self.obs_model.predict(input)
            
            if eta<0:
                usol[0][1] *= -1

            # print(usol)

            usol[0][0] = np.clip(usol[0][0], 0.01, 1)
            usol[0][1] = np.clip(usol[0][1], -0.8, 0.8)
            usol[0][2] = np.clip(usol[0][2], 0.01, 1)

            Pt = 1.0
            Pn = 1.0

            en,et,phi = error(path_segments,self.current_state,self.current_state[3])
                
            usol[0][0]-= Pt*et
            usol[0][1]-= Pn*en

            self.current_state = self.simulate_kinematic_step(self.current_state, usol[0], self.Ts_sim)

            self.closed_loop_trajectory.append(self.current_state[:3])  # Store (x, y, theta)


    def run_NN_pf(self):
        """
        Run the closed-loop control using the path-following neural network (NN).
        """
<<<<<<< HEAD
        self.closed_loop_controls = []  # Store control inputs
        self.solver_times = []  # Store inference times
=======
        closed_loop_controls = []  # Store control inputs
        solver_times = []  # Store inference times
        closed_loop_trajectory = [] 
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

        while np.linalg.norm(self.current_state[:2] - self.final_position[:2]) > 0.1:
            # Prepare inputs for the NN
            x_hat, y_hat, theta_hat, s_hat, eta = T_z(
                path_segments,
                self.current_state[0],
                self.current_state[1],
                self.current_state[2],
                self.current_state[3],
            )

            # Handle orientation flip for eta < 0
            if eta >= 0:
                input_data = [[x_hat, y_hat, theta_hat, s_hat, eta]]
            else:
                input_data = [[x_hat, -y_hat, -theta_hat, s_hat, -eta]]

            # Measure inference time
            start_time = time.time()
            usol = self.predict_policy(input_data)
            end_time = time.time()

<<<<<<< HEAD
            self.solver_times.append(end_time - start_time)
=======
            solver_times.append(end_time - start_time)
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

            # Adjust NN output based on eta
            if eta < 0:
                usol[0, 1] *= -1

            # Clip NN outputs to enforce constraints
            usol = np.clip(usol, [0.01, -0.8, 0.01], [1, 0.8, 1])

            # Simulate the next state
            self.current_state = self.simulate_kinematic_step(self.current_state, usol[0], self.Ts_sim)

            # Log the closed-loop trajectory
<<<<<<< HEAD
            self.closed_loop_trajectory.append(self.current_state.copy())
            # Log the control inputs and s0
            self.closed_loop_controls.append(usol[0].copy())

        # Compute and display inference time statistics
        mean_time = np.mean(self.solver_times)
        std_time = np.std(self.solver_times)
        worst_time = np.max(self.solver_times)
        table = [["Metric", "Time (s)"],
                 ["Mean", f"{mean_time:.6f}"],
                 ["Standard Deviation", f"{std_time:.6f}"],
                 ["Worst Case", f"{worst_time:.6f}"]]
        print(tabulate(table, headers="firstrow", tablefmt="grid"))

    def load_csv_data(self, file_path):
        """Load CSV data for overlay."""
        data = np.loadtxt(file_path, delimiter=",", skiprows=1)
        return data
    
    def analyze_timing_data(self, csv_file):
        """
        Analyze timing data from the CSV and display statistics as a table.
        """
        # Load CSV data
        csv_data = self.load_csv_data(csv_file)
        times = csv_data[:, -1]  # Assuming the last column contains time data

        # Compute statistics
        mean_time = np.mean(times)
        std_time = np.std(times)
        worst_time = np.max(times)

        # Prepare the table
        table = [
            ["Metric", "Time (s)"],
            ["Mean", f"{mean_time:.6f}"],
            ["Standard Deviation", f"{std_time:.6f}"],
            ["Worst Case", f"{worst_time:.6f}"],
        ]

        # Print the table
        print(tabulate(table, headers="firstrow", tablefmt="grid"))

        # Optionally return the statistics for further use
        return {"mean": mean_time, "std": std_time, "worst": worst_time}

=======
            closed_loop_trajectory.append(self.current_state.copy())
            # Log the control inputs and s0
            closed_loop_controls.append(usol[0].copy())
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097

        return closed_loop_trajectory,closed_loop_controls,solver_times

    def plot_trajectory(self, csv_file=None, trajectory=None, label=None, plt_obs=False):
        """
        Plot the trajectory of a closed-loop solution or a CSV file.
        If `csv_file` is provided, plot the CSV data.
        If `trajectory` is provided, plot the given trajectory data.
        If `plt_obs` is True, plot the obstacles from `self.obs_list`.
        """
        if not self.plot_initialized:
            self.initialize_plots()

        if not self.ref_plotted:
            # Plot the reference path
            s_values = np.linspace(0, path_segments[-1].end_time, 500)
            ref_path = np.array([f_s(s).full().flatten()[:2] for s in s_values])
            self.ax_traj.plot(ref_path[:, 0], ref_path[:, 1], 'g-', label="Reference Path")

            # Add a green marker at the start of the reference path
            start_x, start_y = ref_path[0, 0], ref_path[0, 1]
            self.ax_traj.plot(start_x, start_y, 'bo')  # Green circle at start
            # Add text "Start" further above the start point
            self.ax_traj.text(
                start_x, start_y - 0.3, "Start",
                color='black',
                fontsize=18,
                verticalalignment='bottom',
                horizontalalignment='center',
                bbox=dict(facecolor='white', alpha=0.8, edgecolor='none')
            )

            # Add a red marker at the end of the reference path
            end_x, end_y = ref_path[-1, 0], ref_path[-1, 1]
            self.ax_traj.plot(end_x, end_y, 'ro')  # Red circle at end
            # Add text "End" further below the end point
            self.ax_traj.text(
                end_x, end_y + 0.3, "End",
                color='black',
                fontsize=18,
                verticalalignment='top',
                horizontalalignment='center',
                bbox=dict(facecolor='white', alpha=0.8, edgecolor='none')
            )

            self.ref_plotted = True

            if plt_obs:
                # Plot the obstacles from `self.obs_list`
                for obs in self.obs_list:
                    obs_x, obs_y = get_deviated_point(path_segments, obs["s"], obs["d"])
                    circle = plt.Circle((obs_x, obs_y), obs["r"] - self.obs_inflation, color="red", alpha=0.5)
                    self.ax_traj.add_patch(circle)



        if csv_file:
            # Plot the trajectory from a CSV file
            csv_data = self.load_csv_data(csv_file)
            self.ax_traj.plot(
                csv_data[:, 0], csv_data[:, 1],
                label=label or "Gazebo Solution", linestyle="--",color="orange"
            )
        elif trajectory is not None:
            # Plot the given trajectory
            trajectory = np.array(trajectory)
            if trajectory.ndim != 2 or trajectory.shape[1] < 2:
                print(f"Invalid trajectory format: expected 2D array, got shape {trajectory.shape}")
                return
            self.ax_traj.plot(
                trajectory[:, 0], trajectory[:, 1],
                label=label or "Closed-Loop Solution", linestyle="--"
            )
        else:
            print("No trajectory data provided for plotting.")

        # Add labels and grid
        self.ax_traj.set_xlabel("x [m]")
        self.ax_traj.set_ylabel("y [m]")
        self.ax_traj.legend()
        self.ax_traj.grid()

    def plot_states(self, csv_file=None, trajectory=None, label=None, plt_obs=False,real=False):
        """
        Plot the trajectory of a closed-loop solution or a CSV file.
        If `csv_file` is provided, plot the CSV data.
        If `trajectory` is provided, plot the given trajectory data.
        If `plt_obs` is True, plot the obstacles from `self.obs_list`.
        """
        if not self.plot_initialized:
            self.initialize_plots()

        if not self.state_ref_plotted:
            # Plot the reference path
            s_values = np.linspace(0, path_segments[-1].end_time, 500)
            ref_path = np.array([f_s(s).full().flatten()[:2] for s in s_values])
            ref_x = ref_path[:, 0]
            ref_y = ref_path[:, 1]

            self.ax_python_states_x.plot(s_values,ref_x)
            self.ax_python_states_y.plot(s_values,ref_y)

            self.ax_gazebo_states_x.plot(s_values,ref_x)
            self.ax_gazebo_states_y.plot(s_values,ref_y)

            self.state_ref_plotted = True

        if csv_file:
            if not real:
                # Plot the trajectory from a CSV file
                csv_data = self.load_csv_data(csv_file)
                gaz_x = csv_data[:, 0]
                gaz_y = csv_data[:, 1]
                s_vals = csv_data[:, 3]
                self.ax_gazebo_states_x.plot(s_vals,gaz_x)
                self.ax_gazebo_states_y.plot(s_vals,gaz_y)
            else:
                # Plot the trajectory from a CSV file
                csv_data = self.load_csv_data(csv_file)
                gaz_x = csv_data[:, 0]
                gaz_y = csv_data[:, 1]
                s_vals = csv_data[:, 3]

                if "NN" in label:
                    self.ax_gazebo_states_x.plot(s_vals,gaz_x)
                    self.ax_gazebo_states_y.plot(s_vals,gaz_y)
                elif "MPC" in label:
                    self.ax_python_states_x.plot(s_vals,gaz_x)
                    self.ax_python_states_y.plot(s_vals,gaz_y)

        elif trajectory is not None:
            # Plot the given trajectory
            trajectory = np.array(trajectory)
            if trajectory.ndim != 2 or trajectory.shape[1] < 2:
                print(f"Invalid trajectory format: expected 2D array, got shape {trajectory.shape}")
                return
            
            py_x = trajectory[:, 0]
            py_y = trajectory[:, 1]
            s_vals = [state[3] for state in trajectory]

            self.ax_python_states_x.plot(s_vals,py_x)
            self.ax_python_states_y.plot(s_vals,py_y)

        else:
            print("No trajectory data provided for plotting.")

    def plot_error(self, ref_function, csv_file=None, trajectory=None, label=None, real=False):
        """
        Plot Cartesian error vs. path progress parameter (s).
        If `csv_file` is provided, use the CSV data.
        If `trajectory` is provided, use the given trajectory data.
        """
        if csv_file:
            csv_data = self.load_csv_data(csv_file)
            s_values = csv_data[:, 3]  # Assuming s0 is the fourth column

            errors = [
                np.sqrt((csv_data[i, 0] - ref_function(csv_data[i, 3]).full().flatten()[0])**2 +
                        (csv_data[i, 1] - ref_function(csv_data[i, 3]).full().flatten()[1])**2)
                for i in range(len(s_values))
            ]
            self.ax_error.plot(
                s_values, np.array(errors).ravel(),
                label=label or "Gazebo Data",color="orange"
            )
        elif trajectory is not None:
            s_values = [state[3] for state in trajectory]
            errors = [
                np.sqrt((state[0] - ref_function(state[3]).full().flatten()[0])**2 +
                        (state[1] - ref_function(state[3]).full().flatten()[1])**2)
                for state in trajectory
            ]
            self.ax_error.plot(
                s_values, np.array(errors).ravel(),
                label=label or "Closed-Loop Solution"
            )
        else:
            print("No error data provided for plotting.")
            return

        # Set the y-axis to a logarithmic scale
        self.ax_error.set_yscale('log')


    def plot_control_inputs(self, csv_file=None, controls=None, trajectory=None, label=None, real=False,single=False):
        """
        Plot control inputs vs. path progress parameter (s).
        If `csv_file` is provided, plot the CSV data.
        If `controls` and `trajectory` are provided, use the given data.
        """
        # Assign distinct colors to each control input
        v_color = '#d62728'          # red for v
        omega_color = '#9467bd'      # purple for ω
        w_color = 'c'    # purple for w

        if csv_file:
            # Load data from CSV
            if not real:
                if not single:
                    csv_data = self.load_csv_data(csv_file)
                    s_values = csv_data[:, 3]  # Assuming s0 is the fourth column
                    v_values = csv_data[:, 4]  # Assuming v is the fifth column
                    omega_values = csv_data[:, 5]  # Assuming omega is the sixth column
                    w_values = csv_data[:, 6]  # Assuming w is the seventh column

                    self.ax_gazebo_control.plot(s_values, v_values, label=r"v [m/s]("+label+")",linestyle='dashed',color=v_color)
                    self.ax_gazebo_control.plot(s_values, omega_values, label=r"$\omega$ [rad/s]("+label+")",linestyle='dashed',color=omega_color)
                    self.ax_gazebo_control.plot(s_values, w_values, label=r"$w [s^{-1}]$("+label+")",linestyle='dashed',color=w_color)
                else:
                    csv_data = self.load_csv_data(csv_file)
                    s_values = csv_data[:, 3]  # Assuming s0 is the fourth column
                    v_values = csv_data[:, 4]  # Assuming v is the fifth column
                    omega_values = csv_data[:, 5]  # Assuming omega is the sixth column
                    w_values = csv_data[:, 6]  # Assuming w is the seventh column

                    self.ax_control.plot(s_values, v_values, label=r"v [m/s]("+label+")",linestyle='solid',color=v_color)
                    self.ax_control.plot(s_values, omega_values, label=r"$\omega$ [rad/s]("+label+")",linestyle='solid',color=omega_color)
                    self.ax_control.plot(s_values, w_values, label=r"$w [s^{-1}]$("+label+")",linestyle='solid',color=w_color)

            else:
                csv_data = self.load_csv_data(csv_file)
                s_values = csv_data[:, 3]  # Assuming s0 is the fourth column
                v_values = csv_data[:, 4]  # Assuming v is the fifth column
                omega_values = csv_data[:, 5]  # Assuming omega is the sixth column
                w_values = csv_data[:, 6]  # Assuming w is the seventh column

                if not single:

                    if "NN" in label:
                        self.ax_gazebo_control.plot(s_values, v_values, label=r"v [m/s]("+label+")",linestyle='dashed',color=v_color)
                        self.ax_gazebo_control.plot(s_values, omega_values, label=r"$\omega$ [rad/s]("+label+")",linestyle='dashed',color=omega_color)
                        self.ax_gazebo_control.plot(s_values, w_values, label=r"$w [s^{-1}]$("+label+")",linestyle='dashed',color=w_color)
                    elif "MPC" in label:
                        self.ax_python_control.plot(s_values, v_values, label=r"v [m/s]("+label+")",linestyle='solid',color=v_color)
                        self.ax_python_control.plot(s_values, omega_values, label=r"$\omega$ [rad/s]("+label+")",linestyle='solid',color=omega_color)
                        self.ax_python_control.plot(s_values, w_values, label=r"$w [s^{-1}]$("+label+")",linestyle='solid',color=w_color)

                else:
                    csv_data = self.load_csv_data(csv_file)
                    s_values = csv_data[:, 3]  # Assuming s0 is the fourth column
                    v_values = csv_data[:, 4]  # Assuming v is the fifth column
                    omega_values = csv_data[:, 5]  # Assuming omega is the sixth column
                    w_values = csv_data[:, 6]  # Assuming w is the seventh column

                    self.ax_control.plot(s_values, v_values, label=r"v [m/s]("+label+")",linestyle='solid',color=v_color)
                    self.ax_control.plot(s_values, omega_values, label=r"$\omega$ [rad/s]("+label+")",linestyle='solid',color=omega_color)
                    self.ax_control.plot(s_values, w_values, label=r"$w [s^{-1}]$("+label+")",linestyle='solid',color=w_color)

        elif controls is not None and trajectory is not None:
            # Ensure the trajectory and controls lengths match
            s_values = [state[3] for state in trajectory]
            controls = np.array(controls)

            if len(s_values) != controls.shape[0]:
                print(f"Mismatch between s_values ({len(s_values)}) and controls ({controls.shape[0]}).")
                return

            # Plot the control inputs with chosen colors
            self.ax_python_control.plot(
                s_values, controls[:, 0],
                color=v_color,
                label=r"v [m/s]("+label+")"
            )
            self.ax_python_control.plot(
                s_values, controls[:, 1],
                color=omega_color,
                label=r"$\omega$ [rad/s]("+label+")"
            )
            self.ax_python_control.plot(
                s_values, controls[:, 2],
                color=w_color,
                label=r"$w [s^{-1}]$("+label+")"
            )
        else:
            print("No control input data or trajectory provided for plotting.")


    def finalize_plots(self,a,b,c,single=False,Real=False):
        """Add legends and show the combined plot."""
        self.ax_traj.legend(loc='center left',
            fontsize=18,        # Set the font size
            frameon=True,       # Add a box around the legend
            framealpha=0.8,     # Adjust transparency of the legend box
            edgecolor='black'   # Set the border color of the legend box
        )
        if single or Real:
            self.ax_error.legend(loc='lower right',
                fontsize=18,        # Set the font size
                frameon=True,       # Add a box around the legend
                framealpha=0.8,     # Adjust transparency of the legend box
                edgecolor='black'   # Set the border color of the legend box
            )
        else:
            self.ax_error.legend(loc='upper right',
                fontsize=18,        # Set the font size
                frameon=True,       # Add a box around the legend
                framealpha=0.8,     # Adjust transparency of the legend box
                edgecolor='black'   # Set the border color of the legend box
            )

        self.ax_gazebo_control.legend(
            loc='upper left',
            bbox_to_anchor=(1.05, 1),
            fontsize=18,
            frameon=True,
            framealpha=0.8,
            edgecolor='black'
        )

        self.ax_python_control.legend(
            loc='upper left',
            bbox_to_anchor=(1.05, 1),
            fontsize=18,
            frameon=True,
            framealpha=0.8,
            edgecolor='black'
        )

        self.ax_control.legend(
            loc='upper left',
            bbox_to_anchor=(1.05, 1),
            fontsize=18,
            frameon=True,
            framealpha=0.8,
            edgecolor='black'
        )

        # plt.tight_layout()
        self.ax_traj.grid(visible=True)

        self.fig_traj.savefig("plots/"+a, bbox_inches='tight')  # Export to PGF for LaTeX integration
        self.fig_error.savefig("plots/"+b, bbox_inches='tight')  # Export to PGF for LaTeX integration

        # self.fig_states.subplots_adjust(hspace=0.5)
        # self.fig_states.savefig("plots/NN_Replan_Real_PF_states.pgf", bbox_inches='tight')  # Export to PGF for LaTeX integration

        if not single:
            self.fig_control.subplots_adjust(hspace=0.5)
            self.fig_control.savefig("plots/"+c, bbox_inches='tight')  # Export to PGF for LaTeX integration
        else:
            self.fig_control_single.savefig("plots/"+c, bbox_inches='tight')  # Export to PGF for LaTeX integration


        # plt.show()


    def load_csv_data(self, file_path):
        """Load CSV data for overlay."""
        data = np.loadtxt(file_path, delimiter=",", skiprows=1)
        return data
    
    def analyze_timing_data(self, timing_array=None, csv_file=None, label=None):
        """
        Analyze timing data and generate a table of statistics.
        If `timing_array` is provided, use that data.
        If `csv_file` is provided, use the CSV data.
        Otherwise, use internal solver times.
        """
        if timing_array is not None:
            times = timing_array
        elif csv_file:
            csv_data = self.load_csv_data(csv_file)
            times = csv_data[:, -1]  # Assuming the last column contains time data
        else:
            times = self.solver_times

        if len(times)==0:
            print(f"No timing data available for {label or 'Simulation'}.")
            return

        # Compute statistics
        mean_time = np.mean(times)*10**3
        std_time = np.std(times)*10**3
        worst_time = np.max(times)*10**3

        # Prepare the table
        table = [
            ["Metric", "Time (ms)"],
            ["Mean", f"{mean_time:.7f}"],
            ["Standard Deviation", f"{std_time:.7f}"],
            ["Worst Case", f"{worst_time:.7f}"],
        ]

        print(f"\nTiming Data for {label or 'Simulation'}:")
        print(tabulate(table, headers="firstrow", tablefmt="grid"))

        # Optionally return the statistics
        return {"mean": mean_time, "std": std_time, "worst": worst_time}

        
    def analyze_error_data(self, ref_function, csv_file=None, trajectory=None, label=None):
        """
        Analyze Cartesian error data and generate a summary table.
        If `csv_file` is provided, use the CSV data; otherwise, use `trajectory`.
        """
        if csv_file:
            csv_data = self.load_csv_data(csv_file)
            s_values = csv_data[:, 3]  # Assuming s0 is the fourth column
            errors = [
                np.sqrt((csv_data[i, 0] - ref_function(csv_data[i, 3]).full().flatten()[0])**2 +
                        (csv_data[i, 1] - ref_function(csv_data[i, 3]).full().flatten()[1])**2)
                for i in range(len(s_values))
            ]
        elif trajectory is not None:
            trajectory = np.array(trajectory)  # Convert to NumPy array if not already
            s_values = [state[3] for state in trajectory]
            errors = [
                np.sqrt((trajectory[i, 0] - ref_function(s_values[i]).full().flatten()[0])**2 +
                        (trajectory[i, 1] - ref_function(s_values[i]).full().flatten()[1])**2)
                for i in range(len(s_values))
            ]
        else:
            raise ValueError("No valid data provided for error analysis.")

        # Compute statistics
        mean_error = np.mean(errors)
        max_error = np.max(errors)

        # Prepare the table
        table = [
            ["Metric", "Error (m)"],
            ["Mean", f"{mean_error:.3f}"],
            ["Max", f"{max_error:.3f}"],
        ]

        print(f"\nError Data for {label or 'Simulation'}:")
        print(tabulate(table, headers="firstrow", tablefmt="grid"))

        # Optionally return the statistics
        return {"mean_error": mean_error, "max_error": max_error}

    def calculate_and_display_min_distances(self,trajectory=None, csv_file=None):
        """
        Calculate the smallest distances to each obstacle and display them in a table.

        Args:
            trajectory (list of tuples): [(x, y), ...] representing robot positions.
            csv_file (str): Path to the CSV file containing trajectory data.
            path_segments (list): List of path segments.
            obs_list (list of dicts): [{"s": obs_s, "d": obs_d, "r": obs_r}, ...].
            obs_inflation (float): Inflation radius for obstacles.

        Returns:
            None
        """
        if trajectory is not None:
            # Use provided trajectory
            robot_trajectory = np.array(trajectory)[:, :2]
        elif csv_file:
            # Load trajectory from CSV
            csv_data = np.loadtxt(csv_file, delimiter=",", skiprows=1)
            robot_trajectory = csv_data[:, :2]
        else:
            raise ValueError("Either 'trajectory' or 'csv_file' must be provided.")

        min_distances = []

        for obs in self.obs_list:
            # Get the obstacle's actual position
            obs_x, obs_y = get_deviated_point(path_segments, obs["s"], obs["d"])
            obs_r = obs["r"] - self.obs_inflation  # Adjust radius by inflation

            # Calculate distances from the robot trajectory to the obstacle
            distances = [
                np.sqrt((robot_x - obs_x) ** 2 + (robot_y - obs_y) ** 2) - obs_r
                for robot_x, robot_y in robot_trajectory
            ]
            # Append the smallest distance for this obstacle
            min_distances.append(max(0, min(distances)))  # Ensure no negative distances

        # Prepare the table
        table = [["Obstacle ID", "Min Distance (m)"]]
        for i, min_distance in enumerate(min_distances):
            table.append([f"Obstacle {i + 1}", f"{min_distance:.3f}"])

        # Display the table
        print("\nMinimum Distances to Obstacles:")
        print(tabulate(table, headers="firstrow", tablefmt="grid"))
   
    
if __name__ == "__main__":

<<<<<<< HEAD
    # Plot results
    # controller.plot_results(x_opt)

    controller.run_NN_pf()
    controller.plot_closed_loop(csv_file="closed_loop_nn_pf_1.csv")
    controller.analyze_timing_data(csv_file="closed_loop_nn_pf_1.csv")

    # controller.run_mpc_loop()
    # controller.plot_closed_loop(csv_file="closed_loop_mpc_1.csv")
    # controller.analyze_timing_data(csv_file="closed_loop_mpc_1.csv")

=======
    # # MPC sim and Gazebo PF
    # controller1 = NMPCController(r1=0.0,r2=0.0,r3=0.0,infl=0.0)

    # # Initialize plots
    # controller1.initialize_plots()


    # # Run MPC simulation
    # mpc_trajectory, mpc_controls, mpc_times = controller1.run_mpc_loop()
    # if mpc_trajectory and mpc_controls:
    #     controller1.plot_trajectory(
    #         trajectory=mpc_trajectory, 
    #         label="NMPC Policy (Python)"
    #     )
    #     controller1.plot_error(
    #         trajectory=mpc_trajectory, 
    #         ref_function=f_s, 
    #         label="NMPC Policy (Python)"
    #     )
    #     controller1.plot_control_inputs(
    #         controls=mpc_controls, 
    #         trajectory=mpc_trajectory, 
    #         label="NMPC Policy (Python)"
    #     )
    #     controller1.plot_states(
    #         trajectory=mpc_trajectory, 
    #         label="NMPC Policy (Python)")
    # else:
    #     print("NMPC Policy simulation produced no valid data.")

    # # Overlay CSV data for MPC
    # controller1.plot_trajectory(
    #     csv_file="closed_loop_mpc_pf_right.csv", 
    #     label="NMPC Policy (Gazebo)"
    # )
    # controller1.plot_error(
    #     csv_file="closed_loop_mpc_pf_right.csv", 
    #     ref_function=f_s, 
    #     label="NMPC Policy (Gazebo)"
    # )
    # controller1.plot_control_inputs(
    #     csv_file="closed_loop_mpc_pf_right.csv", 
    #     label="NMPC Policy (Gazebo)"
    # )
    # controller1.plot_states(
    #     csv_file="closed_loop_mpc_pf_right.csv", 
    #     label="NMPC Policy (Gazebo)"
    # )

    # # # Summarize NMPC Policy results
    # print("\n=== NMPC Policy Results PF ===")
    # controller1.analyze_timing_data(timing_array=mpc_times, label="NMPC Policy (Python)")
    # controller1.analyze_error_data(ref_function=f_s, trajectory=mpc_trajectory, label="NMPC Policy (Python)")
    # controller1.analyze_timing_data(csv_file="closed_loop_mpc_pf_right.csv", label="NMPC Policy (Gazebo)")
    # controller1.analyze_error_data(ref_function=f_s, csv_file="closed_loop_mpc_pf_right.csv", label="NMPC Policy (Gazebo)")

    # # Finalize plots
    # controller1.finalize_plots("MPC_sim_and_Gazebo_PF_traj_right.pgf","MPC_sim_and_Gazebo_PF_error_right.pgf","MPC_sim_and_Gazebo_PF_control_inputs_right.pgf")


    # # NN sim and Gazebo PF
    # controller2 = NMPCController(r1=0.0,r2=0.0,r3=0.0,infl=0.0)

    # # Initialize plots
    # controller2.initialize_plots()

    # # Run NN simulation
    # nn_trajectory, nn_controls, nn_times = controller2.run_NN_pf()
    # if nn_trajectory and nn_controls:
    #     controller2.plot_trajectory(
    #         trajectory=nn_trajectory, 
    #         label="NN Policy (Python)"
    #     )
    #     controller2.plot_error(
    #         trajectory=nn_trajectory, 
    #         ref_function=f_s, 
    #         label="NN Policy (Python)"
    #     )
    #     controller2.plot_control_inputs(
    #         controls=nn_controls, 
    #         trajectory=nn_trajectory, 
    #         label="NN Policy (Python)"
    #     )
    #     controller2.plot_states(
    #         trajectory=nn_trajectory, 
    #         label="NN Policy (Python)"
    #     )
    # else:
    #     print("NN Policy simulation produced no valid data.")

    # # Overlay CSV data for NN
    # controller2.plot_trajectory(
    #     csv_file="closed_loop_nn_pf_right.csv", 
    #     label="NN Policy (Gazebo)"
    # )
    # controller2.plot_error(
    #     csv_file="closed_loop_nn_pf_right.csv", 
    #     ref_function=f_s, 
    #     label="NN Policy (Gazebo)"
    # )
    # controller2.plot_control_inputs(
    #     csv_file="closed_loop_nn_pf_right.csv", 
    #     label="NN Policy (Gazebo)"
    # )
    # controller2.plot_states(
    #     csv_file="closed_loop_nn_pf_right.csv", 
    #     label="NN Policy (Gazebo)"
    # )

    # # # Summarize NN Policy results
    # print("\n=== NN Policy Results PF ===")
    # controller2.analyze_timing_data(timing_array=nn_times, label="NN Policy (Python)")
    # controller2.analyze_error_data(ref_function=f_s, trajectory=nn_trajectory, label="NN Policy (Python)")
    # controller2.analyze_timing_data(csv_file="closed_loop_nn_pf_right.csv", label="NN Policy (Gazebo)")
    # controller2.analyze_error_data(ref_function=f_s, csv_file="closed_loop_nn_pf_right.csv", label="NN Policy (Gazebo)")

    # # Finalize plots
    # controller2.finalize_plots("NN_sim_and_Gazebo_PF_traj_right.pgf","NN_sim_and_Gazebo_PF_error_right.pgf","NN_sim_and_Gazebo_PF_control_inputs_right.pgf")


    # # MPC sim and Gazebo PF+OA
    # controller3 = NMPCController()

    # # Initialize plots
    # controller3.initialize_plots()

    # # Run MPC simulation
    # mpc_trajectory, mpc_controls, mpc_times = controller3.run_mpc_loop()
    # if mpc_trajectory and mpc_controls:
    #     controller3.plot_trajectory(
    #         trajectory=mpc_trajectory, 
    #         label="NMPC Policy (Python)",plt_obs=True
    #     )
    #     controller3.plot_error(
    #         trajectory=mpc_trajectory, 
    #         ref_function=f_s, 
    #         label="NMPC Policy (Python)"
    #     )
    #     controller3.plot_control_inputs(
    #         controls=mpc_controls, 
    #         trajectory=mpc_trajectory, 
    #         label="NMPC Policy (Python)"
    #     )
    #     controller3.plot_states(
    #         trajectory=mpc_trajectory, 
    #         label="NMPC Policy (Python)"
    #     )
    # else:
    #     print("NMPC Policy simulation produced no valid data.")

    # # Overlay CSV data for MPC
    # controller3.plot_trajectory(
    #     csv_file="closed_loop_mpc_pf_oa_right.csv", 
    #     label="NMPC Policy (Gazebo)",plt_obs=True
    # )
    # controller3.plot_error(
    #     csv_file="closed_loop_mpc_pf_oa_right.csv", 
    #     ref_function=f_s, 
    #     label="NMPC Policy (Gazebo)"
    # )
    # controller3.plot_control_inputs(
    #     csv_file="closed_loop_mpc_pf_oa_right.csv", 
    #     label="NMPC Policy (Gazebo)"
    # )
    # controller3.plot_states(
    #     csv_file="closed_loop_mpc_pf_oa_right.csv", 
    #     label="NMPC Policy (Gazebo)"
    # )


    # # Summarize NMPC Policy results
    # print("\n=== NMPC Policy Results PF+OA ===")
    # controller3.analyze_timing_data(timing_array=mpc_times, label="NMPC Policy (Python)")
    # controller3.analyze_error_data(ref_function=f_s, trajectory=mpc_trajectory, label="NMPC Policy (Python)")
    # controller3.calculate_and_display_min_distances(trajectory=mpc_trajectory)
    
    # controller3.analyze_timing_data(csv_file="closed_loop_mpc_pf_oa_right.csv", label="NMPC Policy (Gazebo)")
    # controller3.analyze_error_data(ref_function=f_s, csv_file="closed_loop_mpc_pf_oa_right.csv", label="NMPC Policy (Gazebo)")
    # controller3.calculate_and_display_min_distances(csv_file="closed_loop_mpc_pf_oa_right.csv")


    # # Finalize plots
    # controller3.finalize_plots("MPC_sim_and_Gazebo_PF_OA_traj_right.pgf","MPC_sim_and_Gazebo_PF_OA_error_right.pgf","MPC_sim_and_Gazebo_PF_OA_control_inputs_right.pgf")


    # # # NN sim and Gazebo PF+OA
    # # # controller4 = NMPCController()

    # # # # Initialize plots
    # # # controller4.initialize_plots()

    # # # # simulation
    # # # nn_trajectory, nn_controls, nn_times = controller4.run_NN_obs()
    # # # if nn_trajectory and nn_controls:
    # # #     controller4.plot_trajectory(
    # # #         trajectory=nn_trajectory, 
    # # #         label="NN Policy (Python)"
    # # #     )
    # # #     controller4.plot_error(
    # # #         trajectory=nn_trajectory, 
    # # #         ref_function=f_s, 
    # # #         label="NN Policy (Python)"
    # # #     )
    # # #     controller4.plot_control_inputs(
    # # #         controls=nn_controls, 
    # # #         trajectory=nn_trajectory, 
    # # #         label="NN Policy (Python)"
    # # #     )
    # # # else:
    # # #     print("NN Policy simulation produced no valid data.")

    # # # # Overlay CSV data for NN
    # # # controller4.plot_trajectory(
    # # #     csv_file="closed_loop_nn_pf.csv", 
    # # #     label="NN Policy (Gazebo)"
    # # # )
    # # # controller4.plot_error(
    # # #     csv_file="closed_loop_nn_pf.csv", 
    # # #     ref_function=f_s, 
    # # #     label="NN Policy (Gazebo)"
    # # # )
    # # # controller4.plot_control_inputs(
    # # #     csv_file="closed_loop_nn_pf.csv", 
    # # #     label="NN Policy (Gazebo)"
    # # # )

    # # # # Summarize NN Policy results
    # # # print("\n=== NN Policy Results ===")
    # # # controller4.analyze_timing_data(timing_array=nn_times, label="NN Policy (Python)")
    # # # controller4.analyze_error_data(ref_function=f_s, trajectory=nn_trajectory, label="NN Policy (Python)")
    # # # controller4.analyze_timing_data(csv_file="closed_loop_nn_pf.csv", label="NN Policy (Gazebo)")
    # # # controller4.analyze_error_data(ref_function=f_s, csv_file="closed_loop_nn_pf.csv", label="NN Policy (Gazebo)")

    # # # # Finalize plots
    # # # controller4.finalize_plots()

    # # Real MPC+NN PF
    # controller5 = NMPCController()

    # # Initialize plots
    # controller5.initialize_plots()

    # # Overlay CSV data for MPC
    # controller5.plot_trajectory(
    #     csv_file="bob_closed_loop_mpc_pf.csv", 
    #     label="NMPC Policy (Real)"
    # )
    # controller5.plot_error(
    #     csv_file="bob_closed_loop_mpc_pf.csv", 
    #     ref_function=f_s, 
    #     label="NMPC Policy (Real)"
    # )
    # controller5.plot_control_inputs(
    #     csv_file="bob_closed_loop_mpc_pf.csv", 
    #     label="NMPC Policy (Real)",real=True
    # )
    # controller5.plot_states(
    #     csv_file="bob_closed_loop_mpc_pf.csv", 
    #     label="NMPC Policy (Real)",real=True
    # )

    # # Overlay CSV data for NN
    # controller5.plot_trajectory(
    #     csv_file="bob_closed_loop_nn_pf_host_comp.csv", 
    #     label="NN Policy (Real)"
    # )
    # controller5.plot_error(
    #     csv_file="bob_closed_loop_nn_pf_host_comp.csv", 
    #     ref_function=f_s, 
    #     label="NN Policy (Real)"
    # )
    # controller5.plot_control_inputs(
    #     csv_file="bob_closed_loop_nn_pf_host_comp.csv", 
    #     label="NN Policy (Real)",real=True
    # )
    # controller5.plot_states(
    #     csv_file="bob_closed_loop_nn_pf_host_comp.csv", 
    #     label="NN Policy (Real)",real=True
    # )

    # # Summarize NN Policy results
    # print("\n=== Real Robot Policy Results ===")
    # controller5.analyze_timing_data(csv_file="bob_closed_loop_mpc_pf.csv", label="NMPC Policy (Real)")
    # controller5.analyze_error_data(ref_function=f_s, csv_file="bob_closed_loop_mpc_pf.csv", label="NMPC Policy (Real)")
    # controller5.analyze_timing_data(csv_file="bob_closed_loop_nn_pf_host_comp.csv", label="NN Policy (Real)")
    # controller5.analyze_error_data(ref_function=f_s, csv_file="bob_closed_loop_nn_pf_host_comp.csv", label="NN Policy (Real)")

    # # Finalize plots
    # # controller5.finalize_plots("MPC_NN_Real_PF_traj.pgf","MPC_NN_Real_PF_error.pgf","MPC_NN_Real_PF_control_inputs.pgf",Real=True)


    # # Real MPC+NN PF+OA
    # controller6 = NMPCController()

    # # Initialize plots
    # controller6.initialize_plots()

    # # Overlay CSV data for MPC
    # controller6.plot_trajectory(
    #     csv_file="bob_closed_loop_mpc_pf_oa_2.csv", 
    #     label="NMPC Policy (Real)",plt_obs=True
    # )
    # controller6.plot_error(
    #     csv_file="bob_closed_loop_mpc_pf_oa_2.csv", 
    #     ref_function=f_s, 
    #     label="NMPC Policy (Real)",real=True
    # )
    # controller6.plot_control_inputs(
    #     csv_file="bob_closed_loop_mpc_pf_oa_2.csv", 
    #     label="NMPC Policy (Real)",real=True,single=True
    # )
    # controller6.plot_states(
    #     csv_file="bob_closed_loop_mpc_pf_oa_2.csv", 
    #     label="NMPC Policy (Real)",real=True
    # )


    # # # Overlay CSV data for NN
    # # controller6.plot_trajectory(
    # #     csv_file="bob_closed_loop_nn_pf_oa_host_comp.csv", 
    # #     label="NN Policy (Real)"
    # # )
    # # controller6.plot_error(
    # #     csv_file="bob_closed_loop_nn_pf_oa_host_comp.csv", 
    # #     ref_function=f_s, 
    # #     label="NN Policy (Real)", real=True
    # # )
    # # controller6.plot_control_inputs(
    # #     csv_file="bob_closed_loop_nn_pf_oa_host_comp.csv", 
    # #     label="NN Policy (Real)", real=True
    # # )

    # # # Summarize NN Policy results
    # print("\n=== Real Robot Policy Results ===")
    # controller6.analyze_timing_data(csv_file="bob_closed_loop_mpc_pf_oa_2.csv", label="NMPC Policy (Real)")
    # controller6.analyze_error_data(ref_function=f_s, csv_file="bob_closed_loop_mpc_pf_oa_2.csv", label="NMPC Policy (Real)")
    # controller6.calculate_and_display_min_distances(csv_file="bob_closed_loop_mpc_pf_oa_2.csv")

    # controller6.analyze_timing_data(csv_file="bob_closed_loop_nn_pf_oa_host_comp.csv", label="NN Policy (Real)")
    # controller6.analyze_error_data(ref_function=f_s, csv_file="bob_closed_loop_nn_pf_oa_host_comp.csv", label="NN Policy (Real)")

    # Finalize plots
    # controller6.finalize_plots("MPC_NN_Real_PF_OA_traj.pgf","MPC_NN_Real_PF_OA_error.pgf","MPC_NN_Real_PF_OA_control_inputs.pgf",single=True)


    # NN Real Replan
    controller7 = NMPCController()

    # Initialize plots
    controller7.initialize_plots()

    # Overlay CSV data for MPC
    controller7.plot_trajectory(
        csv_file="replan_real/bob_closed_loop_nn_pf_replan.csv", 
        label="NMPC Policy Replan (Real)",plt_obs=True
    )
    controller7.plot_error(
        csv_file="replan_real/bob_closed_loop_nn_pf_replan.csv", 
        ref_function=f_s, 
        label="NMPC Policy Replan (Real)",real=True
    )
    controller7.plot_control_inputs(
        csv_file="replan_real/bob_closed_loop_nn_pf_replan.csv", 
        label="NMPC Policy Replan (Real)",real=True,single=True
    )
    controller7.plot_states(
        csv_file="replan_real/bob_closed_loop_nn_pf_replan.csv", 
        label="NMPC Policy Replan (Real)",real=True
    )


    # # # Overlay CSV data for NN
    # # controller6.plot_trajectory(
    # #     csv_file="bob_closed_loop_nn_pf_oa_host_comp.csv", 
    # #     label="NN Policy (Real)"
    # # )
    # # controller6.plot_error(
    # #     csv_file="bob_closed_loop_nn_pf_oa_host_comp.csv", 
    # #     ref_function=f_s, 
    # #     label="NN Policy (Real)", real=True
    # # )
    # # controller6.plot_control_inputs(
    # #     csv_file="bob_closed_loop_nn_pf_oa_host_comp.csv", 
    # #     label="NN Policy (Real)", real=True
    # # )

    # # # Summarize NN Policy results
    print("\n=== Real Robot Policy Results ===")
    controller7.analyze_timing_data(csv_file="replan_real/bob_closed_loop_nn_pf_replan.csv", label="NMPC Policy Replan (Real)")
    controller7.analyze_error_data(ref_function=f_s, csv_file="replan_real/bob_closed_loop_nn_pf_replan.csv", label="NMPC Policy Replan (Real)")
    controller7.calculate_and_display_min_distances(csv_file="replan_real/bob_closed_loop_nn_pf_replan.csv")

    # # # controller6.analyze_timing_data(csv_file="bob_closed_loop_nn_pf_oa_host_comp.csv", label="NN Policy (Real)")
    # # # controller6.analyze_error_data(ref_function=f_s, csv_file="bob_closed_loop_nn_pf_oa_host_comp.csv", label="NN Policy (Real)")

    # # Finalize plots    
    # controller7.finalize_plots("NN_Replan_Real_PF_traj.pgf","NN_Replan_Real_PF_error.pgf","NN_Replan_Real_PF_control_inputs.pgf",single=True)


    # # NN Gazebo Replan
    # controller8 = NMPCController()

    # # Initialize plots
    # controller8.initialize_plots()

    # # Overlay CSV data for MPC
    # controller8.plot_trajectory(
    #     csv_file="replan_sim/closed_loop_nn_pf_right_replan.csv", 
    #     label="NN Replan (Gazebo)",plt_obs=True
    # )
    # controller8.plot_error(
    #     csv_file="replan_sim/closed_loop_nn_pf_right_replan.csv", 
    #     ref_function=f_s, 
    #     label="NN Replan (Gazebo)"
    # )
    # controller8.plot_control_inputs(
    #     csv_file="replan_sim/closed_loop_nn_pf_right_replan.csv", 
    #     label="NN Replan (Gazebo)",real=True,single=True
    # )
    # controller8.plot_states(
    #     csv_file="replan_sim/closed_loop_nn_pf_right_replan.csv", 
    #     label="NN Replan (Gazebo)",real=True
    # )


    # # # # Overlay CSV data for NN
    # # # controller8.plot_trajectory(
    # # #     csv_file="bob_closed_loop_nn_pf_oa_host_comp.csv", 
    # # #     label="NN Policy (Real)"
    # # # )
    # # # controller8.plot_error(
    # # #     csv_file="bob_closed_loop_nn_pf_oa_host_comp.csv", 
    # # #     ref_function=f_s, 
    # # #     label="NN Policy (Real)", real=True
    # # # )
    # # # controller8.plot_control_inputs(
    # # #     csv_file="bob_closed_loop_nn_pf_oa_host_comp.csv", 
    # # #     label="NN Policy (Real)", real=True
    # # # )

    # # # Summarize NN Policy results
    # print("\n=== Real Robot Policy Results ===")
    # controller8.analyze_timing_data(csv_file="replan_sim/closed_loop_nn_pf_right_replan.csv", label="NN Replan (Gazebo)")
    # controller8.analyze_error_data(ref_function=f_s, csv_file="replan_sim/closed_loop_nn_pf_right_replan.csv", label="NN Replan (Gazebo)")
    # controller8.calculate_and_display_min_distances(csv_file="replan_sim/closed_loop_nn_pf_right_replan.csv")

    # # controller8.analyze_timing_data(csv_file="bob_closed_loop_nn_pf_oa_host_comp.csv", label="NN Policy (Real)")
    # # controller8.analyze_error_data(ref_function=f_s, csv_file="bob_closed_loop_nn_pf_oa_host_comp.csv", label="NN Policy (Real)")

    # # Finalize plots    
    # controller8.finalize_plots("NN_Replan_Gazebo_PF_traj_right.pgf","NN_Replan_Gazebo_PF_error_right.pgf","NN_Replan_Gazebo_PF_control_inputs_right.pgf",single=True)

>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097
