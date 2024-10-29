
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import casadi as ca
import random
import control as ct

from .waypoint_filter import *
from .line_fitting import *
from .transform import *
from .path_segments import *
from .plotting import *


def LSPB_fit(waypoints,n,epsilon,v_max):

    # Get the selected waypoints and their original indices
    original_indices, selected_waypoints = select_waypoints_with_indices(waypoints, n)

    # Get the indices and the processed waypoints
    processed_indices, processed_waypoints = preprocess_segments_by_deviation(selected_waypoints, original_indices)

    path_segments = []

    # Initialize previous right epsilon and its time
    x_eps_right_prev, y_eps_right_prev = None, None

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

            if processed_indices[i] - processed_indices[i-1] > 20:

                s_p = p1
                m = (y_eps_left-s_p[1])/(x_eps_left-s_p[0])

                while processed_indices[i] - processed_indices[i-1] - n*k > 20:
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

            if processed_indices[i] - processed_indices[i-1] > 20:

                s_p = (x_eps_right_prev, y_eps_right_prev)
                m = (y_eps_left-s_p[1])/(x_eps_left-s_p[0])

                while processed_indices[i] - processed_indices[i-1] - n*k > 20:
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

            if processed_indices[i] - processed_indices[i-1] > 20:

                s_p = (x_eps_right, y_eps_right)
                m = (final_waypoint[1]-s_p[1])/(final_waypoint[0]-s_p[0])

                while processed_indices[i] - processed_indices[i-1] - n*k > 20:
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

# # Define the CasADi variable for s
# s = ca.MX.sym('s')

# # Select the correct segment and calculate (x, y) for the given value of s
# selected_result = f(path_segments, s)

# # Define a CasADi function to evaluate the selected (x, y) for a given s
# f_s = ca.Function('f_s', [s], [selected_result])

# T_z_with_random_s_per_segment_with_transform_plot(path_segments, f_s,v_max,epsilon)

# s = 1.0          # Example value for forward speed (adjust based on your control inputs)
# phi_hat = 0.5    # Example orientation angle in radians (adjust as necessary)
# Delta_t = 0.1    # Discrete time step in seconds (adjust as needed)

# # State transition matrix A
# A = np.array([
#     [1, 0, -s * np.sin(phi_hat) * Delta_t, 0],
#     [0, 1,  s * np.cos(phi_hat) * Delta_t, 0],
#     [0, 0, 1, 0],
#     [0, 0, 0, 1]
# ])

# # Control matrix B
# B = np.array([
#     [np.cos(phi_hat) * Delta_t, 0, 0],
#     [np.sin(phi_hat) * Delta_t, 0, 0],
#     [0, Delta_t, 0],
#     [0, 0, Delta_t]
# ])

# Q=0
# R=0

# P,L,K = ct.dare(A,B,Q,R)

# K = -np.array(K)

# fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

# for i, segment in enumerate(path_segments):

#     if segment.segment_type == 'line':
#         s = np.linspace(segment.start_time, segment.end_time-0.01, 1000)
#         x_vals = []
#         y_vals = []

#         for s_value in s:
#             result = f_s(s_value)
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
#             result = f_s(s_value)
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

#     print(segment.eta)

# plt.show()

