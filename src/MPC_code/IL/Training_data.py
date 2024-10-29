import numpy as np

# Define constants and parameters
curvatures = np.linspace(0, 3, 11)      # Curvature values η_k from 0 to 3
d_k = 0.3                                  # Controls the length of the path segment
theta_steps = 10                         # Number of discrete points along θ for each path
grid_resolution = 10                     # Number of samples along each axis (tangential, normal, orientation)
Delta_t = 0.1                            # Time step for discrete dynamics

v_max = 0.1

def gamma(eta):
    if eta==0:
        return 1
    else:
        return 0.5*(np.sqrt(1+(2*eta)**2)+((np.arcsinh(2*eta))/(2*eta)))

def g(v_max,eta):
    return v_max/gamma(eta)

# Function to generate a parabolic path given η
def generate_path(eta):
    g_eta = g(v_max,eta)
    theta_vals = np.linspace(-g_eta*d_k, -g_eta*d_k, theta_steps)  # Adjusted theta range
    path_points = [(g_eta * theta, eta * (g_eta * theta)**2) for theta in theta_vals]
    return theta_vals, np.array(path_points)

# Placeholder for solving the optimal control problem for a given state and path
def compute_optimal_control(state, path_segment):
    # Use an OCP solver like CasADi to calculate the optimal control (s, ω, v)
    # For now, we'll just return some dummy values for illustration
    s = np.random.uniform(0, 1)
    omega = np.random.uniform(-0.5, 0.5)
    v = np.random.uniform(0, 1)
    return s, omega, v

# Lists to store training data
states = []
controls = []

# Loop over each curvature value η_k to create the training data
for eta in curvatures:
    theta_vals, path_points = generate_path(eta)

    # For each point θ_k,i on the path, create an orthogonal cuboid of poses
    for theta, (px, py) in zip(theta_vals, path_points):
        # Sample poses around each point (px, py) within the cuboid
        tangential_range = np.linspace(-0.5, 0.5, grid_resolution)  # Tangential offsets
        normal_range = np.linspace(-0.5, 0.5, grid_resolution)      # Normal offsets
        orientation_range = np.linspace(-0.1, 0.1, grid_resolution) # Orientation offsets

        for dx in tangential_range:
            for dy in normal_range:
                for dphi in orientation_range:
                    # Compute tangential and normal adjustments
                    tangent_adjustment = np.array([dx, 0])         # Tangential direction along x-axis
                    normal_adjustment = np.array([0, dy])          # Normal direction along y-axis

                    # Adjust the base point (px, py) by tangential and normal offsets
                    adjusted_px, adjusted_py = px + tangent_adjustment[0], py + normal_adjustment[1]

                    # Define the current state in the transformed frame
                    state = [
                        adjusted_px,     # Transformed x-position (with tangential adjustment)
                        adjusted_py,     # Transformed y-position (with normal adjustment)
                        dphi,            # Orientation offset
                        theta,           # Path progress
                        eta              # Curvature parameter
                    ]

                    # Solve the optimal control problem to find control actions
                    s, omega, v = compute_optimal_control(state, (px, py))

                    # Store the state and control pairs in the dataset
                    states.append(state)
                    controls.append([s, omega, v])

# Convert to numpy arrays for training
states = np.array(states)
controls = np.array(controls)

# Save the data or use it directly for training
np.save("train_states.npy", states)
np.save("train_controls.npy", controls)
