import numpy as np
import matplotlib.pyplot as plt
from transform import *
from line_fitting import *

# Plot the original parabola and the lines before and after
def plot_parabola_and_lines(ax, coefficients, p1, p2, prev_start, prev_end, next_start, next_end):
    a, b, c, d, e, f = coefficients

    # Plot the straight line for the previous segment (from prev_start to prev_end)
    if prev_start is not None and prev_end is not None:
        x_left = [prev_start[0], prev_end[0]]
        y_left = [prev_start[1], prev_end[1]]  # Straight line from prev_start to prev_end
        ax.plot(x_left, y_left, 'r-', linewidth=2, label='Previous Line Segment')

    # Plot the straight line for the next segment (from next_start to next_end)
    if next_start is not None and next_end is not None:
        x_right = [next_start[0], next_end[0]]
        y_right = [next_start[1], next_end[1]]  # Straight line from next_start to next_end
        ax.plot(x_right, y_right, 'g-', linewidth=2, label='Next Line Segment')


    # Plot the start and end points of the parabola
    ax.scatter(p1[0], p1[1], color='blue', s=100, zorder=5, label='Start Point')  # Start point marker
    ax.scatter(p2[0], p2[1], color='blue', s=100, zorder=5, label='End Point')    # End point marker

    # Set x-limits and y-limits based on the line segment points
    all_x = [p1[0], p2[0]]
    all_y = [p1[1], p2[1]]

    if prev_start is not None:
        all_x.append(prev_start[0])
        all_y.append(prev_start[1])

    if prev_end is not None:
        all_x.append(prev_end[0])
        all_y.append(prev_end[1])

    if next_start is not None:
        all_x.append(next_start[0])
        all_y.append(next_start[1])

    if next_end is not None:
        all_x.append(next_end[0])
        all_y.append(next_end[1])

    # Now, plot the parabola using the coefficients (between p1 and p2)
    x_orig = np.linspace(min(all_x) - 1, max(all_x) + 1, 400)  # Limit the range strictly between p1 and p2
    y_orig = np.linspace(min(all_y) - 1, max(all_y) + 1, 400)
    X_orig, Y_orig = np.meshgrid(x_orig, y_orig)
    Z_orig = a * X_orig**2 + b * X_orig * Y_orig + c * Y_orig**2 + d * X_orig + e * Y_orig + f
    ax.contour(X_orig, Y_orig, Z_orig, levels=[0], colors='blue')


    ax.set_xlim(min(all_x) - 1, max(all_x) + 1)
    ax.set_ylim(min(all_y) - 1, max(all_y) + 1)


def T_z_with_random_s_per_segment_with_transform_plot(path_segments, f_s,v_max,eps):
    """
    Function to plot two random s values for each segment and visualize both the original and transformed positions.
    Displays each plot after selecting two random s values for each segment.
    """
    
    # Define the orientations based on the current segments for both s values
    def get_orientation(segment, x, y):
        if segment.segment_type == 'line':
            if find_grad(segment) > 0:
                return np.arctan(find_grad(segment))
            else:
                return np.arctan(find_grad(segment)) + np.pi
            
        elif segment.segment_type == 'parabola':
            tf_points = segment.inv_transform_p((x, y))
            return np.arctan(2 * segment.eta * tf_points[0]) + segment.rotation

    # For each segment, pick two random `s` values and plot
    for i, segment in enumerate(path_segments):
        # Create the figure and axes for each iteration
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

        s = np.linspace(segment.start_time, segment.end_time, 1000)
        x_vals = []
        y_vals = []

        for s_value in s:
            result = f_s(s_value)
            x_vals.append(float(result[0]))
            y_vals.append(float(result[1]))

        ax1.plot(x_vals, y_vals, 'b-', label="Segment")
        ax1.set_title(f"Original Segment and Input Positions")
        ax1.set_xlabel('x')
        ax1.set_ylabel('y')
        ax1.grid(True)

        ax1.scatter(segment.start_point[0], segment.start_point[1], color='green', s=100, zorder=5, label=f"start")
        ax1.scatter(segment.end_point[0], segment.end_point[1], color='red', s=100, zorder=5, label=f"end")

        tfx = []
        tfy = []

        for i in range(len(x_vals)):
            point = segment.inv_transform_p((x_vals[i],y_vals[i]))
            tfx.append(point[0])
            tfy.append(point[1])

        ax2.plot(tfx,tfy,'k-')

        transformed_start = segment.transformed_startpoint()
        transformed_end = segment.transformed_endpoint()

        ax2.scatter(transformed_start[0], transformed_start[1], color='green', s=100, zorder=5, label=f"start")
        ax2.scatter(transformed_end[0], transformed_end[1], color='red', s=100, zorder=5, label=f"end")

        # Generate two random values of `s` within the current segment's time bounds
        s_random_1 = np.random.uniform(segment.start_time, segment.end_time)
        s_random_2 = np.random.uniform(segment.start_time, segment.end_time)

        # Assign the smaller `s` value to green and larger `s` value to red
        if s_random_1 < s_random_2:
            s_smaller, s_larger = s_random_1, s_random_2
            color_smaller, color_larger = 'green', 'red'
        else:
            s_smaller, s_larger = s_random_2, s_random_1
            color_smaller, color_larger = 'green', 'red'

        # Get the current positions by evaluating f_s for these random s values
        current_position_smaller = f_s(s_smaller).full().flatten()[:2] 
        current_position_larger = f_s(s_larger).full().flatten()[:2]
        x_noisy_smaller, y_noisy_smaller = current_position_smaller[0] + np.random.normal(0, 0.01), current_position_smaller[1] + np.random.normal(0, 0.01)
        x_noisy_larger, y_noisy_larger = current_position_larger[0] + np.random.normal(0, 0.01), current_position_larger[1] + np.random.normal(0, 0.01)

        # x_noisy_smaller, y_noisy_smaller = current_position_smaller[0] , current_position_smaller[1]
        # x_noisy_larger, y_noisy_larger = current_position_larger[0] , current_position_larger[1] 

        # Define the current orientations based on the segment type
        current_orientation_smaller = get_orientation(segment, x_noisy_smaller, y_noisy_smaller)
        current_orientation_larger = get_orientation(segment, x_noisy_larger, y_noisy_larger)

        # Plot on the left and right plots using T_z for the smaller s
        transformed_x_smaller, transformed_y_smaller, transformed_orientation_smaller, _ = T_z(
            path_segments, x_noisy_smaller, y_noisy_smaller, current_orientation_smaller, s_smaller
        )

        # Plot on the left and right plots using T_z for the larger s
        transformed_x_larger, transformed_y_larger, transformed_orientation_larger, _ = T_z(
            path_segments, x_noisy_larger, y_noisy_larger, current_orientation_larger, s_larger
        )

        # Plot the points on both plots
        ax1.scatter(x_noisy_smaller, y_noisy_smaller, color=color_smaller, s=100, zorder=5, label=f"Input Position (s_small, Segment {i+1})")
        ax1.scatter(x_noisy_larger, y_noisy_larger, color=color_larger, s=100, zorder=5, label=f"Input Position (s_large, Segment {i+1})")

        ax2.scatter(transformed_x_smaller, transformed_y_smaller, color=color_smaller, s=100, zorder=5, label=f"Transformed Position (s_small, Segment {i+1})")
        ax2.scatter(transformed_x_larger, transformed_y_larger, color=color_larger, s=100, zorder=5, label=f"Transformed Position (s_large, Segment {i+1})")

        # Set different arrow properties for left and right plots
        arrow_length_left = 0.15  # Larger arrow length for left plot
        arrow_length_right = 0.08  # Smaller arrow length for right plot
        arrow_head_size_left = 0.03  # Larger arrowhead size for left plot
        arrow_head_size_right = 0.01  # Smaller arrowhead size for right plot

        # Plot the original orientation on ax1 with a larger arrow
        ax1.arrow(x_noisy_smaller, y_noisy_smaller, arrow_length_left * np.cos(current_orientation_smaller), arrow_length_left * np.sin(current_orientation_smaller),
                  head_width=arrow_head_size_left, head_length=arrow_head_size_left, fc=color_smaller, ec=color_smaller, label=f"Original Orientation (s_small, Segment {i+1})")
        ax1.arrow(x_noisy_larger, y_noisy_larger, arrow_length_left * np.cos(current_orientation_larger), arrow_length_left * np.sin(current_orientation_larger),
                  head_width=arrow_head_size_left, head_length=arrow_head_size_left, fc=color_larger, ec=color_larger, label=f"Original Orientation (s_large, Segment {i+1})")

        # Plot the transformed orientation for the smaller s point on ax2 with a smaller arrow
        ax2.arrow(transformed_x_smaller, transformed_y_smaller, arrow_length_right * np.cos(transformed_orientation_smaller), arrow_length_right * np.sin(transformed_orientation_smaller),
                  head_width=arrow_head_size_right, head_length=arrow_head_size_right, fc=color_smaller, ec=color_smaller, label=f"Transformed Orientation (s_small, Segment {i+1})")

        # Plot the transformed orientation for the larger s point on ax2 with a smaller arrow
        ax2.arrow(transformed_x_larger, transformed_y_larger, arrow_length_right * np.cos(transformed_orientation_larger), arrow_length_right * np.sin(transformed_orientation_larger),
                  head_width=arrow_head_size_right, head_length=arrow_head_size_right, fc=color_larger, ec=color_larger, label=f"Transformed Orientation (s_large, Segment {i+1})")
            

        cof = segment.eta
        mid = (segment.end_time - segment.start_time)/2

        px_hat = g(v_max,cof)*np.linspace(- mid, segment.end_time - segment.start_time - mid, 100) 
        py_hat = cof*px_hat**2

        ax2.plot(px_hat, py_hat, 'b--', linewidth=2, label="g(eta)*s^2")

        # Add labels and grid to the transformed plot
        ax2.set_title(f"Transformed Segment and Positions")
        ax2.set_xlabel('x')
        ax2.set_ylabel('y')
        ax2.grid(True)

        # Adjust the limits to match both plots if necessary
        ax1.set_xlim(min(x_vals) - 0.1, max(x_vals) + 0.1)
        ax1.set_ylim(min(y_vals) - 0.1, max(y_vals) + 0.1)

        # # Add legends to each plot and display the plot
        ax1.legend()
        ax2.legend()

        # Show the plot for this iteration before moving on to the next segment
        plt.show()