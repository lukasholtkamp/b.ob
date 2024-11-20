import numpy as np
import matplotlib.pyplot as plt
from sympy import *

def rotation_angle(A, B, C):
    # If A equals C, then the parabola rotates by 45 degrees
    if A == C:
        return np.pi / 4  # 45 degrees in radians
    
    ratio = (A - C) / B
    theta = 0.5 * np.arctan(1 / ratio)
    return theta


def rotate_parabola(A, B, C, D, E, F, theta):
    # Rotate the parabola by adjusting A', C' such that one is forced to zero
    
    A_prime = A * np.cos(theta)**2 + B * np.sin(theta) * np.cos(theta) + C * np.sin(theta)**2
    C_prime = A * np.sin(theta)**2 - B * np.sin(theta) * np.cos(theta) + C * np.cos(theta)**2

    
    # Calculate B_prime (should be 0 for a parabola)
    B_prime = 0
    
    # Rotate the linear terms
    D_prime = D * np.cos(theta) + E * np.sin(theta)
    E_prime = E * np.cos(theta) - D * np.sin(theta)
    
    # F remains the same
    F_prime = F
    
    return A_prime, B_prime, C_prime, D_prime, E_prime, F_prime

# Function to calculate the vertex
def calculate_vertex(A_prime, D_prime, E_prime, F_prime):
    a = -A_prime / E_prime
    b = -D_prime / E_prime
    c = -F_prime / E_prime
    x_v = -b / (2 * a)
    y_v = a*(x_v)**2+b*x_v+c
    return x_v, y_v

# Function to calculate the transformation matrix (rotation + translation)
def transformation_matrix(theta, tx, ty):
    return np.array([
                [np.cos(theta), -np.sin(theta), tx],
                [np.sin(theta), np.cos(theta), ty],
                [0, 0, 1]
            ])

def get_transformation_matrix(p1, p2):
    """
    Returns a 3x3 transformation matrix that rotates a line segment defined by p1 and p2 to be
    parallel to the x-axis and translates it so the endpoints are equidistant from the y-axis.

    :param p1: The first endpoint of the line (x1, y1).
    :param p2: The second endpoint of the line (x2, y2).
    :return: A 3x3 transformation matrix.
    """
    # Calculate the angle theta between the line and the x-axis
    delta_x = p2[0] - p1[0]
    delta_y = p2[1] - p1[1]
    theta = np.arctan2(delta_y,delta_x)

    # Rotation matrix (3x3) to align with the x-axis
    R = np.array([[np.cos(-theta), -np.sin(-theta), 0],
                  [np.sin(-theta),  np.cos(-theta), 0],
                  [0,               0,              1]])

    # Find the midpoint of the original line
    midpoint_x = (p1[0] + p2[0]) / 2
    midpoint_y = (p1[1] + p2[1]) / 2

    # After rotation, we want to translate the line such that its midpoint is on the y-axis
    distance = np.linalg.norm([p2[0] - p1[0], p2[1] - p1[1]]) / 2
    
    # Translation to move midpoint to (0, 0) on the x-axis
    T = np.array([[1, 0, -midpoint_x],
                  [0, 1, -midpoint_y],
                  [0, 0, 1]])

    # Combine rotation and translation (final transformation matrix)
    M = R @ T  # First translate, then rotate



    return M, theta

def apply_transformation(point, M):
    """
    Apply a 3x3 transformation matrix to a point.
    
    :param point: Tuple (x, y) representing the point.
    :param M: 3x3 transformation matrix.
    :return: Transformed point (x', y').
    """
    # Convert point to homogeneous coordinates (x, y, 1)
    point_h = np.array([point[0], point[1], 1])
    
    # Apply the transformation matrix
    transformed_point_h = M @ point_h
    
    # Convert back from homogeneous coordinates
    return transformed_point_h[:2]

def find_grad(segment):
    p1 = segment.start_point
    p2 = segment.end_point

    return (p2[1]-p1[1])/(p2[0]-p1[0])

def get_orientation(segment, x, y):
    if segment.segment_type == 'line':
        if find_grad(segment) > 0:
            return np.arctan(find_grad(segment))
        else:
            return np.arctan(find_grad(segment)) + np.pi
        
    elif segment.segment_type == 'parabola':
        tf_points = segment.inv_transform_p((x, y))
        return np.arctan(2 * segment.eta * tf_points[0]) + segment.rotation

def visualize_errors(path_segments, current_state, s, en, et):
    """
    Visualize the current position, path point at f(s), and error vectors.

    Parameters:
    - path_segments: List of path segment objects representing the global path.
    - current_state: Current state of the robot as (x, y, theta).
    - s: Current path parameter value.
    - en: Normal error.
    - et: Tangent error.
    - reference_traj_func: Function that returns the (x, y) position on the path at a given s.
    """
    # Calculate tangent and normal vectors for visualization
    current_segment = None

    for segment in path_segments:
        if segment.start_time <= s <= segment.end_time:
            current_segment = segment
            break

    if s>path_segments[-1].end_time:
        current_segment = path_segments[-1]
    

    # Get the point on the path at f(s)
    path_point = current_segment.f(s)
    path_x, path_y = float(path_point[0]), float(path_point[1])

    # Plot the path points for visualization (optional)
    for segment in path_segments:
        s_vals = np.linspace(segment.start_time, segment.end_time, 100)
        segment_points = [segment.f(s_val) for s_val in s_vals]
        segment_points = np.array(segment_points, dtype=float)
        plt.plot(segment_points[:, 0], segment_points[:, 1], 'k-', alpha=0.5)  # Plot the path

    # Plot the current position and path point
    plt.plot(current_state[0], current_state[1], 'ro', label="Current Position")  # Red dot for current position
    plt.plot(path_x, path_y, 'go', label="Path Point at f(s)")  # Green dot for path point

    if current_segment:
        tangent_orientation = get_orientation(current_segment, path_x, path_y)
        tangent_vector = np.array([np.cos(tangent_orientation), np.sin(tangent_orientation)])
        normal_vector = np.array([-tangent_vector[1], tangent_vector[0]])

        # Normalize the vectors for scaling
        tangent_vector = tangent_vector / np.linalg.norm(tangent_vector)
        normal_vector = normal_vector / np.linalg.norm(normal_vector)

        # Scale vectors for visualization
        et_vector = et * tangent_vector
        en_vector = en * normal_vector

        # Plot tangent and normal error vectors
        plt.arrow(path_x, path_y, et_vector[0], et_vector[1], color='b', head_width=0.05, label="Tangent Error (e_t)")
        plt.arrow(path_x, path_y, en_vector[0], en_vector[1], color='c', head_width=0.05, label="Normal Error (e_n)")

    # Add labels and legends
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Visualization of Tangent and Normal Errors")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')  # Ensure equal scaling for x and y axes
    plt.show()

def error(path_segments,current_state,s):

    current_segment = None

    for segment in path_segments:
        if segment.start_time <= s <= segment.end_time:
            current_segment = segment
            break
    
    if s>path_segments[-1].end_time:
        current_segment = path_segments[-1]

    # Get the point on the path at f(s)
    path_point = current_segment.f(s)
    path_x, path_y = float(path_point[0]), float(path_point[1])

    # Calculate the position error vector
    error_vector = np.array([current_state[0] - path_x, current_state[1] - path_y])

    # Calculate the orientation of the tangent at (path_x, path_y)
    tangent_orientation = get_orientation(current_segment, path_x, path_y)

    # Create the tangent vector from the orientation
    tangent_vector = np.array([np.cos(tangent_orientation), np.sin(tangent_orientation)])

    # Normalize the tangent vector
    tangent_vector = tangent_vector / np.linalg.norm(tangent_vector)

    # Calculate the normal vector as perpendicular to the tangent vector
    normal_vector = np.array([-tangent_vector[1], tangent_vector[0]])

    # Project the error vector onto the tangent and normal vectors
    et = np.dot(error_vector, tangent_vector)
    en = np.dot(error_vector, normal_vector)

    return en,et,tangent_orientation

def get_deviated_point(path_segments, s, deviation):
    # Find the current segment based on the value of s
    current_segment = None
    for segment in path_segments:
        if segment.start_time <= s <= segment.end_time:
            current_segment = segment
            break
    
    if s > path_segments[-1].end_time:
        current_segment = path_segments[-1]

    # Get the point on the path at s
    path_point = current_segment.f(s)
    path_x, path_y = float(path_point[0]), float(path_point[1])

    # Calculate the orientation of the tangent at (path_x, path_y)
    tangent_orientation = get_orientation(current_segment, path_x, path_y)

    # Create the tangent vector from the orientation
    tangent_vector = np.array([np.cos(tangent_orientation), np.sin(tangent_orientation)])

    # Normalize the tangent vector
    tangent_vector = tangent_vector / np.linalg.norm(tangent_vector)

    # Calculate the normal vector as perpendicular to the tangent vector
    normal_vector = np.array([-tangent_vector[1], tangent_vector[0]])

    # Apply the deviation in the normal direction
    deviated_x = path_x + deviation * normal_vector[0]
    deviated_y = path_y + deviation * normal_vector[1]

    return deviated_x, deviated_y


def T_z(path_segments, x, y, current_orientation, s):
    """
    Transforms the current position (x, y) and orientation based on the path segment corresponding to s.
    Plots the original and transformed segment, position, and orientation.
    
    Parameters:
    - path_segments: List of segments in the path.
    - x: Current x-coordinate of the position.
    - y: Current y-coordinate of the position.
    - current_orientation: Current orientation (in radians).
    - s: The current s value (parameter along the path).

    Returns:
    - (transformed_x, transformed_y, transformed_orientation, s): Transformed position and orientation, along with the given s.
    """
    
    # Find the segment corresponding to the current value of s
    current_segment = None
    for segment in path_segments:
        if segment.start_time <= s <= segment.end_time:
            current_segment = segment
            break

    if s > path_segments[-1].end_time:
        current_segment = path_segments[-1]
    
    if current_segment is None:
        raise ValueError(f"s={s} does not fall within any segment's time bounds.")
    
    # Transform the current position (x, y) using the segment's transformation matrix
    transformed_position = current_segment.inv_transform_p((x,y))
    transformed_x = transformed_position[0]
    transformed_y = transformed_position[1]

    # Adjust the orientation based on the segment's rotation
    transformed_orientation = current_orientation - current_segment.rotation  # Assuming segment has a 'rotation' attribute

    mid = (segment.end_time - segment.start_time)/2

    s_transformed = s - segment.start_time - mid

    return transformed_x, transformed_y, transformed_orientation, s_transformed, current_segment.eta

def T_z_obs(path_segments, x, y, current_orientation, s, obs_x, oby_y):
    """
    Transforms the current position (x, y) and orientation based on the path segment corresponding to s.
    Plots the original and transformed segment, position, and orientation.
    
    Parameters:
    - path_segments: List of segments in the path.
    - x: Current x-coordinate of the position.
    - y: Current y-coordinate of the position.
    - current_orientation: Current orientation (in radians).
    - s: The current s value (parameter along the path).

    Returns:
    - (transformed_x, transformed_y, transformed_orientation, s): Transformed position and orientation, along with the given s.
    """
    
    # Find the segment corresponding to the current value of s
    current_segment = None
    for segment in path_segments:
        if segment.start_time <= s <= segment.end_time:
            current_segment = segment
            break

    if s > path_segments[-1].end_time:
        current_segment = path_segments[-1]
    
    if current_segment is None:
        raise ValueError(f"s={s} does not fall within any segment's time bounds.")
    
    # Transform the current position (x, y) using the segment's transformation matrix
    transformed_position = current_segment.inv_transform_p((x,y))
    transformed_x = transformed_position[0]
    transformed_y = transformed_position[1]

    transformed_obs = current_segment.inv_transform_p((obs_x,oby_y))
    transformed_obs_x = transformed_obs[0]
    transformed_obs_y = transformed_obs[1]

    # Adjust the orientation based on the segment's rotation
    transformed_orientation = current_orientation - current_segment.rotation  # Assuming segment has a 'rotation' attribute

    mid = (segment.end_time - segment.start_time)/2

    s_transformed = s - segment.start_time - mid

    return transformed_x, transformed_y, transformed_orientation, s_transformed, current_segment.eta,transformed_obs_x,transformed_obs_y

def gamma(eta):
    if eta==0:
        return 1
    else:
        return 0.5*(np.sqrt(1+(2*eta)**2)+((np.arcsinh(2*eta))/(2*eta)))

def g(v_max,eta):
    return v_max/gamma(eta)



