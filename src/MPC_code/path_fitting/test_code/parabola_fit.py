import sympy as sp
import numpy as np
import matplotlib.pyplot as plt

## Function to generate random points and slopes
def generate_random_params():
    A, B = -0.07481773, -0.02175956 # Random left epsilon point
    E, F = 0.4383632, -0.0011148  # Random center epsilon point
    C, D = 0.90954756, 0.14311929  # Random right epsilon point

    S1 = (F - B) / (E - A) if (E - A) != 0 else float('inf')
    S2 = (D - F) / (C - E) if (C - E) != 0 else float('inf')

    return A, B, C, D, S1, S2

# Function to calculate the intersection point of two lines
def find_intersection(A, B, S1, C, D, S2):
    x_intersection = (D - B - S2 * C + S1 * A) / (S1 - S2)
    y_intersection = S1 * (x_intersection - A) + B
    return x_intersection, y_intersection

# Function to automatically adjust the plot range
def adjust_plot_range(A, B, C, D, x_int, y_int, margin=2):
    x_min = min(A, C, x_int) - margin
    x_max = max(A, C, x_int) + margin
    y_min = min(B, D, y_int) - margin
    y_max = max(B, D, y_int) + margin
    return (x_min, x_max), (y_min, y_max)

# Function to check if a point (x, y) is inside a triangle defined by points P1, P2, P3
def is_point_in_triangle(P1, P2, P3, P):
    # Using the cross-product method to check if the point is inside the triangle
    def sign(p1, p2, p3):
        return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])

    d1 = sign(P, P1, P2)
    d2 = sign(P, P2, P3)
    d3 = sign(P, P3, P1)

    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)

    return not (has_neg and has_pos)

# Function to plot the lines, points, and implicit curve inside the triangle mask
def plot_lines_and_curve_with_triangle_mask(A, B, S1, C, D, S2, a, b, c, d, e, f):
    # Find the intersection of the two lines
    x_int, y_int = find_intersection(A, B, S1, C, D, S2)

    x_range, y_range = adjust_plot_range(A, B, C, D, x_int, y_int)
    
    # Generate meshgrid over a reasonable range that includes the triangle
    x_vals = np.linspace(min(A, C, x_int) - 1, max(A, C, x_int) + 1, 400)
    y_vals = np.linspace(min(B, D, y_int) - 1, max(B, D, y_int) + 1, 400)
    X, Y = np.meshgrid(x_vals, y_vals)

    # Define triangle vertices: (A, B), (C, D), and the intersection
    P1 = np.array([A, B])
    P2 = np.array([C, D])
    P3 = np.array([x_int, y_int])

    # Calculate the equation of the implicit curve
    Z = a * X**2 + b * X * Y + c * Y**2 + d * X + e * Y + f

    # Create a mask for points inside the triangle
    inside_triangle = np.zeros_like(X, dtype=bool)

    # Check if each point in the meshgrid is inside the triangle
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            P = np.array([X[i, j], Y[i, j]])
            if is_point_in_triangle(P1, P2, P3, P):
                inside_triangle[i, j] = True

    # Apply the mask to the Z values to keep the implicit curve inside the triangle
    Z[~inside_triangle] = np.nan  # Mask out values outside the triangle

    # Generate x values for the straight lines
    x_vals_lines = np.linspace(min(A, C, x_int) - 1, max(A, C, x_int) + 1, 400)

    # Equation of the first line: y = S1 * (x - A) + B
    y_vals_1 = S1 * (x_vals_lines - A) + B

    # Equation of the second line: y = S2 * (x - C) + D
    y_vals_2 = S2 * (x_vals_lines - C) + D

    # Plot the lines
    plt.plot(x_vals_lines, y_vals_1, label=f'Line through ({A:.2f}, {B:.2f}) with slope {S1:.2f}', color='blue')
    plt.plot(x_vals_lines, y_vals_2, label=f'Line through ({C:.2f}, {D:.2f}) with slope {S2:.2f}', color='green')

    # Plot the points A, B, C, D, and the intersection point
    plt.scatter([A, C, x_int], [B, D, y_int], color='black', zorder=5, label=f'Points ({A:.2f}, {B:.2f}), ({C:.2f}, {D:.2f}), Intersection')

    # Plot the masked contour for the implicit curve
    plt.contour(X, Y, Z, levels=[0], colors='red', label="Implicit Curve")

    # Set wider limits and equal scaling on both axes
    plt.xlim(x_range)
    plt.ylim(y_range)
    plt.gca().set_aspect('equal', adjustable='box')

    # Labeling the plot
    plt.title('Lines and Implicit Curve within Triangle Mask')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.axhline(0, color='black', linewidth=0.5)
    plt.axvline(0, color='black', linewidth=0.5)
    plt.grid(True)
    plt.legend()

# Example usage with random points and slopes
A, B, C, D, S1, S2 = generate_random_params()
a = 1  # Example coefficient for x^2 term

# Define symbolic variables for x and y
x, y = sp.symbols('x y')

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

# Rearrange the equation in the form of a quadratic equation in x and y
equation_expanded = sp.expand(equation)

# Extract the coefficients for the general conic equation
coeff_x2_value = equation_expanded.coeff(x, 2)
coeff_xy_value = equation_expanded.coeff(x).coeff(y)
coeff_y2_value = equation_expanded.coeff(y, 2)
coeff_x_value = equation_expanded.coeff(x).subs({x: 0, y: 0})
coeff_y_value = equation_expanded.coeff(y).subs({x: 0, y: 0})
constant_term_value = equation.subs({x: 0, y: 0})

# Convert coefficients to numerical values
b = float(coeff_xy_value)
c = float(coeff_y2_value)
d = float(coeff_x_value)
e = float(coeff_y_value)
f = float(constant_term_value)

# print(a)
# print(b)
# print(c)
# print(d)
# print(e)
# print(f)


# Plot the lines and implicit curve with random points and slopes
plot_lines_and_curve_with_triangle_mask(A, B, S1, C, D, S2, a, b, c, d, e, f)

# Function to find the a
plt.show()

