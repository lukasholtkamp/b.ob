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

    return transformed_x, transformed_y, transformed_orientation, s_transformed

def gamma(eta):
    if eta==0:
        return 1
    else:
        return 0.5*(np.sqrt(1+(2*eta)**2)+((np.arcsinh(2*eta))/(2*eta)))

def g(v_max,eta):
    return v_max/gamma(eta)

