import numpy as np
import matplotlib.pyplot as plt

# Function to calculate the angle of rotation
def rotation_angle(A, B, C):
    return 0.5 * np.arctan2(B, A - C)

# Function to calculate the transformation matrix (rotation + translation)
def transformation_matrix(theta, tx, ty):
    return np.array([
        [np.cos(-theta), -np.sin(-theta), tx],
        [np.sin(-theta), np.cos(-theta), ty],
        [0, 0, 1]
    ])

# Function to perform the transformation (rotation + translation) on a point
def transform_point(x, y, matrix):
    point = np.array([x, y, 1])  # Homogeneous coordinates [x, y, 1]
    transformed_point = np.dot(matrix, point)  # Apply the transformation
    return transformed_point[:2]  # Return the transformed x, y

# Function to rotate the parabola coefficients
def rotate_parabola(A, B, C, D, E, F, theta):
    A_prime = (A + C)  # The rotated parabola's coefficient for x'^2 or y'^2
    D_prime = (D * np.sqrt(np.abs(A)) + E * np.sqrt(np.abs(C))) / np.sqrt(A + C)
    E_prime = (E * np.sqrt(np.abs(A)) - D * np.sqrt(np.abs(C))) / np.sqrt(A + C)
    return A_prime, D_prime, E_prime, F

# Function to calculate the vertex
def calculate_vertex(A_prime, D_prime, E_prime, F_prime):
    a = -A_prime / E_prime
    b = -D_prime / E_prime
    c = -F_prime / E_prime
    x_v = -b / (2 * a)
    y_v = (-b**2 - 4 * a * c) / (4 * a)
    return x_v, y_v

def inverse_rotate_point(x, y, theta):
    x_rot = x * np.cos(-theta) + y * np.sin(-theta)
    y_rot = -x * np.sin(-theta) + y * np.cos(-theta)
    return x_rot, y_rot

# Interactive plotting class
class InteractiveTransformationPlot:
    def __init__(self, theta, A, B, C, D, E, F, A_prime, D_prime, E_prime, F_prime, vertex_translation):
        self.theta = theta
        self.A, self.B, self.C, self.D, self.E, self.F = A, B, C, D, E, F
        self.A_prime, self.D_prime, self.E_prime, self.F_prime = A_prime, D_prime, E_prime, F_prime

        # Create the combined rotation and translation matrix
        self.transformation = transformation_matrix(theta, -vertex_translation[0], -vertex_translation[1])

        # Create figure and axes for both plots
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 6))
        self.ax1.set_title("Original Parabola")
        self.ax2.set_title("Transformed Parabola (Vertex at (0,0))")
        self.ax1.set_xlabel('x')
        self.ax1.set_ylabel('y')
        self.ax2.set_xlabel('x')
        self.ax2.set_ylabel('y')

        x_original_v, y_original_v = inverse_rotate_point(vertex_translation[0], vertex_translation[1], theta)

        # Original parabola plot
        x_orig = np.linspace(-10, 10, 400)
        y_orig = np.linspace(-10, 10, 400)
        self.X_orig, self.Y_orig = np.meshgrid(x_orig, y_orig)
        self.Z_orig = self.A * self.X_orig**2 + self.B * self.X_orig * self.Y_orig + self.C * self.Y_orig**2 + self.D * self.X_orig + self.E * self.Y_orig + self.F
        self.ax1.contour(self.X_orig, self.Y_orig, self.Z_orig, levels=[0], colors='blue')
        self.ax1.plot(x_original_v, y_original_v, 'ko', label="Inverse Rotated Vertex (-theta)")

        # Transformed parabola plot
        x_trans = np.linspace(-10, 10, 400)
        y_trans = np.linspace(-10, 10, 400)
        self.X_trans, self.Y_trans = np.meshgrid(x_trans, y_trans)
        self.Z_trans = self.A_prime * self.X_trans**2 + self.D_prime * self.X_trans + self.E_prime * self.Y_trans + self.F_prime
        self.ax2.contour(self.X_trans-vertex_translation[0], self.Y_trans-vertex_translation[1], self.Z_trans, levels=[0], colors='green')
        self.ax2.plot(0, 0, 'ko', label="Transformed Vertex at (0, 0)")

        # Initialize plots for interactive points
        self.orig_point = self.ax1.plot([], [], 'ro')[0]
        self.trans_point = self.ax2.plot([], [], 'ro')[0]

        # Connect mouse motion event
        self.cid = self.fig.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)

    def on_mouse_move(self, event):
        # If the event occurs within the bounds of the original plot (left)
        if event.inaxes == self.ax1:
            # Get the current mouse position in data coordinates
            x, y = event.xdata, event.ydata

            # Apply the transformation matrix (rotation + translation)
            transformed_x, transformed_y = transform_point(x, y, self.transformation)

            # Update the position of the points
            self.orig_point.set_data(x, y)
            self.trans_point.set_data(transformed_x, transformed_y)

            # Redraw the figure to show the updated points
            self.fig.canvas.draw()

# Example coefficients
A = 1
B = 2
C = 1
D = -3
E = -4
F = 0

# Calculate the rotation angle
theta = rotation_angle(A, B, C)

# Rotate the parabola coefficients
A_prime, D_prime, E_prime, F_prime = rotate_parabola(A, B, C, D, E, F, theta)

# Calculate the vertex of the rotated parabola
vertex_x, vertex_y = calculate_vertex(A_prime, D_prime, E_prime, F_prime)

# Create the interactive transformation plot with the combined rotation and translation
interactive_plot = InteractiveTransformationPlot(theta, A, B, C, D, E, F, A_prime, D_prime, E_prime, F_prime, (vertex_x, vertex_y))

# Show the interactive plot
plt.show()
