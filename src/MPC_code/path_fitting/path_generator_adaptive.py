
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
    
path_segments = LSPB_fit(20,0.15,0.1)


# Define the CasADi variable for s
s = ca.MX.sym('s')

# Select the correct segment and calculate (x, y) for the given value of s
selected_result = f(path_segments, s)

# Define a CasADi function to evaluate the selected (x, y) for a given s
f_s = ca.Function('f_s', [s], [selected_result])


# Plotting Code with Safety Radius for Point-wise Intersection Check and Highlighting Closest Endpoints Outside
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

# Define obstacle properties
obs_x, obs_y, obs_r = 2, 0.2, 0.17
safety_radius = 0.18  # Define the safety radius

# Draw the obstacle with safety radius as a circle
total_radius = obs_r + safety_radius
circle = plt.Circle((obs_x, obs_y), obs_r, color='blue', fill=False)
ax1.add_patch(circle)

# Loop through path segments and plot points only if they do not intersect the obstacle
for segment in path_segments:
    s = np.linspace(segment.start_time, segment.end_time - 0.01, 1000)
    x_vals = []
    y_vals = []

    intersection_detected = False  # To track if any point intersects with the obstacle

    for s_value in s:
        # Evaluate the point on the segment using the f_s function
        result = f_s(s_value)
        x, y = float(result[0]), float(result[1])

        # Calculate distance from the point to the circle center
        distance = np.linalg.norm(np.array([x, y]) - np.array([obs_x, obs_y]))

        # Append the point only if it does not fall within the obstacle + safety radius
        if distance > total_radius:
            x_vals.append(x)
            y_vals.append(y)
        else:
            intersection_detected = True

    # Plotting the segment points that do not intersect with the obstacle
    if len(x_vals) > 0:
        color = 'r-' if segment.segment_type == 'line' else 'b-'  # Red for lines, Blue for parabolas
        ax1.plot(x_vals, y_vals, color, label="Full Path using f_s")
        ax1.set_title("Original Segment and Input Positions")
        ax1.set_xlabel('x')
        ax1.set_ylabel('y')
        ax1.grid(True)

        # Highlight closest start or endpoint that is outside the obstacle
        if intersection_detected:
            start_point = np.array(segment.start_point)
            end_point = np.array(segment.end_point)
            obstacle_center = np.array([obs_x, obs_y])

            # Calculate distances to the obstacle center
            distance_to_start = np.linalg.norm(start_point - obstacle_center)
            distance_to_end = np.linalg.norm(end_point - obstacle_center)

            # Determine the closest point that is outside the obstacle radius
            closest_point = None
            if distance_to_start > total_radius and distance_to_end > total_radius:
                closest_point = start_point if distance_to_start < distance_to_end else end_point
            elif distance_to_start > total_radius:
                closest_point = start_point
            elif distance_to_end > total_radius:
                closest_point = end_point

            # Highlight the closest point in red
            if closest_point is not None:
                ax1.plot(closest_point[0], closest_point[1], 'ro', markersize=8)

        # Mark the start and end points
        ax1.plot(segment.start_point[0], segment.start_point[1], 'ko')
        ax1.plot(segment.end_point[0], segment.end_point[1], 'ko')

        # Inverse transform plotting (optional)
        tfx = []
        tfy = []

        for i in range(len(x_vals)):
            point = segment.inv_transform_p((x_vals[i], y_vals[i]))
            tfx.append(point[0])
            tfy.append(point[1])

        ax2.plot(tfx, tfy, 'g-')

plt.show()

