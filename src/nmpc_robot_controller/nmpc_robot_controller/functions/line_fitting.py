import numpy as np
import sympy as sp

# Define the straight line equation between two points
def line_eq_between_points(p1, p2):
    slope = (p2[1] - p1[1]) / (p2[0] - p1[0]) if p2[0] != p1[0] else float('inf')  # Prevent division by zero
    intercept = p1[1] - slope * p1[0]
    return [slope,intercept]

def interpolate_time(p1, p2, epsilon_point, time_at_p1, time_at_p2):
    """
    Interpolates the time for an epsilon point between two waypoints (p1, p2).
    
    :param p1: Coordinates of the first waypoint (x1, y1).
    :param p2: Coordinates of the second waypoint (x2, y2).
    :param epsilon_point: Coordinates of the epsilon point (x_eps, y_eps).
    :param time_at_p1: Time at the first waypoint.
    :param time_at_p2: Time at the second waypoint.
    :return: Extrapolated time at the epsilon point.
    """
    # Calculate distances
    total_distance = np.linalg.norm(np.array(p2) - np.array(p1))
    distance_to_epsilon = np.linalg.norm(np.array(epsilon_point) - np.array(p1))
    
    # Proportion of the distance from p1 to the epsilon point
    proportion = distance_to_epsilon / total_distance
    
    # Interpolated time at the epsilon point
    time_at_epsilon = time_at_p1 + proportion * (time_at_p2 - time_at_p1)
    
    return time_at_epsilon

# Function to calculate epsilon points on lines
def calculate_epsilon_points_on_line(p_left, p_curr, p_right, eps_left, eps_right):
    x_curr = p_curr[0]
    y_curr = p_curr[1]

    x_left = p_left[0]
    y_left = p_left[1]

    x_right  = p_right[0] 
    y_right = p_right[1]

    # Slopes for left and right lines
    m_left = (y_curr - y_left) / (x_curr - x_left) if (x_curr - x_left) != 0 else float('inf')
    m_right = (y_right - y_curr) / (x_right - x_curr) if (x_right - x_curr) != 0 else float('inf')

    # Left epsilon point
    if m_left != float('inf'):
        delta_x_left = eps_left / np.sqrt(1 + m_left**2)
        delta_y_left = m_left * delta_x_left
        x_eps_left = x_curr - delta_x_left if x_left < x_curr else x_curr + delta_x_left
        y_eps_left = y_curr - delta_y_left if x_left < x_curr else y_curr + delta_y_left
    else:
        x_eps_left = x_curr
        y_eps_left = y_curr - eps_left

    # Right epsilon point
    if m_right != float('inf'):
        delta_x_right = eps_right / np.sqrt(1 + m_right**2)
        delta_y_right = m_right * delta_x_right
        x_eps_right = x_curr + delta_x_right if x_curr < x_right else x_curr - delta_x_right
        y_eps_right = y_curr + delta_y_right if x_curr < x_right else y_curr - delta_y_right
    else:
        x_eps_right = x_curr
        y_eps_right = y_curr + eps_right

    return x_eps_left, y_eps_left, x_eps_right, y_eps_right

def fit_parabola_to_epsilon_points(A, B, C, D, S1, S2):
    # Define symbolic variables
    x, y = sp.symbols('x y')

    # Example coefficient for x^2 term (adjust as needed for fitting the parabola)
    a = 1

    # Define the quadratic equation
    equation = (
        (a * x - (A * (S1 + S2) - C * (S1 + S2) - 2 * B + 2 * D) * a /
        (2 * A * S1 * S2 - 2 * C * S1 * S2 - B * (S1 + S2) + D * (S1 + S2)) * y) ** 2
        - 2 * (
            2 * A**3 * S1**2 * S2**2 + 2 * C**3 * S1**2 * S2**2
            - 2 * (2 * S1**2 * S2 + S1 * S2**2) * C**2 * D
            + 2 * (S1**2 + 2 * S1 * S2) * C * D**2
            - 2 * D**3 * S1 - 2 * B**3 * S2
            - 2 * (C * S1**2 * S2**2 - D * S1 * S2**2) * A**2
            + (2 * (2 * S1 * S2 + S2**2) * A + (S1**2 - 2 * S1 * S2 - S2**2) * C - 2 * D * (S1 - 2 * S2)) * B**2
            - (2 * C**2 * S1**2 * S2**2 - 4 * C * D * S1**2 * S2 + (S1**2 + 2 * S1 * S2 - S2**2) * D**2) * A
            + (2 * C**2 * S1**2 * S2 - 2 * (S1**2 * S2 + 2 * S1 * S2**2) * A**2
            - (3 * S1**2 + 2 * S1 * S2 - S2**2) * C * D + 2 * D**2 * (2 * S1 - S2)
            + (4 * C * S1 * S2**2 + (S1**2 - 2 * S1 * S2 - 3 * S2**2) * D) * A) * B
        ) * a**2 / (
            4 * A**2 * S1**2 * S2**2 + 4 * C**2 * S1**2 * S2**2
            + (S1**2 + 2 * S1 * S2 + S2**2) * B**2
            - 4 * (S1**2 * S2 + S1 * S2**2) * C * D
            + (S1**2 + 2 * S1 * S2 + S2**2) * D**2
            - 4 * (2 * C * S1**2 * S2**2 - (S1**2 * S2 + S1 * S2**2) * D) * A
            - 2 * (2 * (S1**2 * S2 + S1 * S2**2) * A - 2 * (S1**2 * S2 + S1 * S2**2) * C
                + (S1**2 + 2 * S1 * S2 + S2**2) * D) * B
        ) * x
        + 2 * (
            2 * A**3 * S1**2 * S2 + 2 * C**3 * S1 * S2**2
            - 2 * (2 * S1 * S2 + S2**2) * C**2 * D
            + 2 * C * D**2 * (S1 + 2 * S2)
            - (2 * (2 * S1**2 * S2 - S1 * S2**2) * C - (S1**2 + 2 * S1 * S2 - S2**2) * D) * A**2
            + 2 * (A * (2 * S1 + S2) - C * S1 + D) * B**2
            - 2 * B**3 - 2 * D**3
            + (2 * (S1**2 * S2 - 2 * S1 * S2**2) * C**2
            - (S1**2 - 2 * S1 * S2 - 3 * S2**2) * C * D - 2 * D**2 * S2) * A
            - (2 * (S1**2 + 2 * S1 * S2) * A**2 + (S1**2 - 2 * S1 * S2 - S2**2) * C**2
            + 4 * C * D * S2 - ((3 * S1**2 + 2 * S1 * S2 - S2**2) * C - 4 * D * S1) * A - 2 * D**2) * B
        ) * a**2 / (
            4 * A**2 * S1**2 * S2**2 + 4 * C**2 * S1**2 * S2**2
            + (S1**2 + 2 * S1 * S2 + S2**2) * B**2
            - 4 * (S1**2 * S2 + S1 * S2**2) * C * D
            + (S1**2 + 2 * S1 * S2 + S2**2) * D**2
            - 4 * (2 * C * S1**2 * S2**2 - (S1**2 * S2 + S1 * S2**2) * D) * A
            - 2 * (2 * (S1**2 * S2 + S1 * S2**2) * A - 2 * (S1**2 * S2 + S1 * S2**2) * C
                + (S1**2 + 2 * S1 * S2 + S2**2) * D) * B
        ) * y
        + (
            4 * (C * S1**2 * S2**2 - D * S1**2 * S2) * A**3 - 4 * (C * S2 - D) * B**3
            - (8 * C**2 * S1**2 * S2**2 - 4 * (3 * S1**2 * S2 + S1 * S2**2) * C * D
            + (3 * S1**2 + 6 * S1 * S2 - S2**2) * D**2) * A**2
            + ((S1**2 - 6 * S1 * S2 - 3 * S2**2) * C**2 + 4 * C * D * (S1 + 3 * S2)
            + 4 * ((2 * S1 * S2 + S2**2) * C - D * (2 * S1 + S2)) * A - 8 * D**2) * B**2
            + 4 * (C**3 * S1**2 * S2**2 - (2 * S1**2 * S2 + S1 * S2**2) * C**2 * D
                + (S1**2 + 2 * S1 * S2) * C * D**2 - D**3 * S1) * A
            - 2 * (
                2 * C**3 * S1 * S2**2 - 2 * (2 * S1 * S2 + S2**2) * C**2 * D
                + 2 * C * D**2 * (S1 + 2 * S2)
                + 2 * ((S1**2 * S2 + 2 * S1 * S2**2) * C - (S1**2 + 2 * S1 * S2) * D) * A**2
                - 2 * D**3
                - (2 * (S1**2 * S2 + 3 * S1 * S2**2) * C**2 - (3 * S1**2 + 10 * S1 * S2 + 3 * S2**2) * C * D
                + 2 * D**2 * (3 * S1 + S2)) * A
            ) * B
        ) * a**2 / (
            4 * A**2 * S1**2 * S2**2 + 4 * C**2 * S1**2 * S2**2
            + (S1**2 + 2 * S1 * S2 + S2**2) * B**2
            - 4 * (S1**2 * S2 + S1 * S2**2) * C * D
            + (S1**2 + 2 * S1 * S2 + S2**2) * D**2
            - 4 * (2 * C * S1**2 * S2**2 - (S1**2 * S2 + S1 * S2**2) * D) * A
            - 2 * (2 * (S1**2 * S2 + S1 * S2**2) * A - 2 * (S1**2 * S2 + S1 * S2**2) * C
                + (S1**2 + 2 * S1 * S2 + S2**2) * D) * B
        )
    )

    # Expand the equation
    equation_expanded = sp.expand(equation)

    # Extract the coefficients for the quadratic equation
    coeff_x2 = equation_expanded.coeff(x, 2)
    coeff_xy = equation_expanded.coeff(x).coeff(y)
    coeff_y2 = equation_expanded.coeff(y, 2)
    coeff_x = equation_expanded.coeff(x).subs({x: 0, y: 0})
    coeff_y = equation_expanded.coeff(y).subs({x: 0, y: 0})
    constant_term = equation.subs({x: 0, y: 0})

    # Convert to numerical values
    coeffs = {
        'a': float(coeff_x2),
        'b': float(coeff_xy),
        'c': float(coeff_y2),
        'd': float(coeff_x),
        'e': float(coeff_y),
        'f': float(constant_term)
    }

    return coeffs

def find_grad(segment):
    p1 = segment.start_point
    p2 = segment.end_point

    if (p2[0]-p1[0]) == 0:
        return 10**6
    else:
        return (p2[1]-p1[1])/(p2[0]-p1[0])