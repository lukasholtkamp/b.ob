import numpy as np
import pandas as pd

# Deviation check function
def deviation_is_small(p1, p2, x_threshold=0.05, y_threshold=0.05):
    x_deviation = abs(p2[0] - p1[0])  # Deviation in x between p1 and p2
    y_deviation = abs(p2[1] - p1[1])  # Deviation in y between p1 and p2
    return x_deviation < x_threshold or y_deviation < y_threshold

# Preprocessing function to detect small deviations and merge segments until the condition is violated
def preprocess_segments_by_deviation(waypoints, original_indices, x_threshold=0.05, y_threshold=0.05):
    new_waypoints = [waypoints[0]]  # Start with the first waypoint
    selected_indices = [original_indices[0]]  # Start with the first original index
    i = 0
    while i < len(waypoints) - 1:
        p1 = waypoints[i]
        p2 = waypoints[i + 1]

        # Compare between p1 and p2 first
        if not deviation_is_small(p1, p2, x_threshold, y_threshold):
            # If deviation between p1 and p2 is large, add p2 as a new waypoint
            new_waypoints.append(p2)
            selected_indices.append(original_indices[i + 1])  # Use the original index of the selected waypoint
            i += 1  # Move to p2 as the start of the next segment
            continue

        # If deviation is small, continue to check further points
        current_segment_end = p2
        j = i + 1
        while j < len(waypoints) - 1:
            next_point = waypoints[j + 1]
            if deviation_is_small(current_segment_end, next_point, x_threshold, y_threshold):
                # Continue merging segments
                current_segment_end = next_point
                j += 1
            else:
                # Stop merging and add the current_segment_end as a new waypoint
                break

        # Add the current_segment_end as a new waypoint
        new_waypoints.append(current_segment_end)
        selected_indices.append(original_indices[j])  # Use the original index
        i = j  # Move to the next point after the merged segment

    return np.array(selected_indices), np.array(new_waypoints)

def select_waypoints_with_indices(waypoints, n):

    # Get the indices of the selected waypoints
    indices = [0] + list(range(n, len(waypoints), n))
    
    # Ensure the last point is added only if it's not already in the indices
    if indices[-1] != len(waypoints) - 1:
        indices.append(len(waypoints) - 1)

    # Extract the corresponding waypoints using the selected indices
    selected_waypoints = np.array(waypoints[indices])

    # Return indices, selected waypoints, and the total number of waypoints
    return indices, selected_waypoints