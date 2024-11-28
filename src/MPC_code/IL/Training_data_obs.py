import numpy as np
import os
import gc  # Garbage collection to free memory
import matplotlib.pyplot as plt

from NMPC_solver_oa import *

# Define constants and parameters
curvatures = np.linspace(0, 2, 11)
d_k = 0.2
theta_steps = 10
grid_resolution = 11
width = 0.4
length = 0.5
v_max = 0.1

inflation = 0.18
cir_r = 0.17
bot_r = 0.23
impact_region = 0.3
cases = np.linspace(0.65, -0.65, 7).tolist()
cases.insert(0, None)
segments = 30
layers = 4
outer_radius = cir_r + impact_region
eps = bot_r / 2

BATCH_SIZE = 100  # Process states in batches of 100

# File paths for data
state_file = "train_states_obs.npy"
control_file = "train_controls_obs.npy"

# Initialize or load arrays
def initialize_data(state_file, control_file):
    if os.path.exists(state_file):
        states = np.load(state_file)
    else:
        states = np.empty((0, 8))  # State array: [x, y, orientation, s, curvature, obs_x, obs_y, obs_r]

    if os.path.exists(control_file):
        controls = np.load(control_file)
    else:
        controls = np.empty((0, 2))  # Control array: [v, omega]

    # Check for consistent lengths
    if len(states) != len(controls):
        min_length = min(len(states), len(controls))
        print(f"Inconsistent lengths detected. Truncating to {min_length}.")
        states = states[:min_length]
        controls = controls[:min_length]

        # Save truncated data back to disk
        save_data(state_file, control_file, states, controls)

    return states, controls

# Save arrays incrementally
def save_data(state_file, control_file, states, controls):
    np.save(state_file, states)
    np.save(control_file, controls)

# Function to calculate dynamic slope and position functions
def calculate_functions(curvature, v_max):
    g_val = g(v_max, curvature)
    m_normal = lambda s: -1 / (2 * curvature * s * g_val) if curvature != 0 else 0
    y_normal = lambda s, x: m_normal(s) * x - m_normal(s) * s * g_val + curvature * (s * g_val) ** 2 if curvature != 0 else 0
    return m_normal, y_normal

# Function to compute the gamma and g functions
def gamma(eta):
    if eta == 0:
        return 1
    else:
        return 0.5 * (np.sqrt(1 + (2 * eta) ** 2) + ((np.arcsinh(2 * eta)) / (2 * eta)))

def g(v_max, eta):
    return v_max / gamma(eta)

# Generate path
def generate_path(eta):
    g_eta = g(v_max, eta)
    theta_vals = np.linspace(-d_k / g_eta, d_k / g_eta, theta_steps)
    path_points = [(g_eta * theta, eta * (g_eta * theta) ** 2) for theta in theta_vals]
    return theta_vals, np.array(path_points)

# Load existing data or initialize new ones
states, controls = initialize_data(state_file, control_file)

# Use the length of the `states` array as the progress tracker
processed_states = 0
print(f"Resuming from state index: {processed_states}")

# Batch containers
batch_states = []
batch_controls = []

# Main loop
for curvature in curvatures:
    m_normal, y_normal = calculate_functions(curvature, v_max)
    th, zeta = generate_path(curvature)

    for case in cases:
        for k in range(len(th)):
            print(f"Processing state index {processed_states}, curvature {curvature}, case {case}, theta {th[k]}")

            try:
                if case is not None:
                    if curvature > 0:
                        dx = case * np.cos(np.arctan(m_normal(th[k])))
                        xc = zeta[k, 0] - dx
                        yc = y_normal(th[k], xc)
                    else:
                        dx = case
                        xc = zeta[k, 0]
                        yc = -dx

                    angles = np.linspace(0, 2 * np.pi, segments, endpoint=False)
                    radii = np.linspace(cir_r + eps, outer_radius, layers)

                    for r in radii:
                        for angle in angles:
                            x = xc + r * np.cos(angle)
                            y = yc + r * np.sin(angle)
                            start_angle = np.arctan(2 * curvature * th[k] * g(v_max, curvature))
                            orientations = np.linspace(start_angle, start_angle + 2 * np.pi, grid_resolution, endpoint=False)
                            orientations %= 2 * np.pi

                            for orientation in orientations:
                                if len(states) > processed_states:
                                    print(f"Skipping already processed state at index {processed_states}")
                                    processed_states += 1
                                    continue

                                state = [x, y, orientation, th[k], curvature, xc, yc, cir_r + inflation]
                                obs_list = [[xc, yc, cir_r + inflation]]

                                _, u_step = run_open_loop_mpc(v_max, [x, y, orientation], th[k], curvature, obs_list)

                                # Add to batch
                                batch_states.append(state)
                                batch_controls.append([u_step[0, 0], u_step[0, 1]])

                                # Process and save batch
                                if len(batch_states) >= BATCH_SIZE:
                                    states = np.vstack((states, batch_states))
                                    controls = np.vstack((controls, batch_controls))

                                    save_data(state_file, control_file, states, controls)

                                    batch_states.clear()
                                    batch_controls.clear()
                                    gc.collect()  # Free memory

                                processed_states += 1

                elif case is None:
                    empty_obs = []
                    tangent_axis = np.linspace(th[k] - length / 2, th[k] + length / 2, grid_resolution)
                    normal_axis = np.linspace(0, width / 2, int(grid_resolution / 2) + 1)

                    for i in range(len(tangent_axis)):
                        for j in range(len(normal_axis)):
                            start_angle = np.arctan(2 * curvature * (tangent_axis[i]) * g(v_max, curvature))
                            angles = np.linspace(start_angle, start_angle + 2 * np.pi, grid_resolution, endpoint=False)
                            angles %= 2 * np.pi

                            for orientation in angles:
                                if len(states) > processed_states:
                                    print(f"Skipping already processed state at index {processed_states}")
                                    processed_states += 1
                                    continue

                                dx = np.abs(normal_axis[j] * np.cos(np.arctan(m_normal(tangent_axis[i]))))
                                x_l = tangent_axis[i] * g(v_max, curvature) - dx
                                x_r = tangent_axis[i] * g(v_max, curvature) + dx

                                for x in [x_l, x_r]:
                                    y_val = y_normal(tangent_axis[i], x)
                                    state = [x, y_val, orientation, th[k], curvature, 0, 0, 0]
                                    _, u_step = run_open_loop_mpc(v_max, [x, y_val, orientation], th[k], curvature, empty_obs)

                                    # Add to batch
                                    batch_states.append(state)
                                    batch_controls.append([u_step[0, 0], u_step[0, 1]])

                                    # Process and save batch
                                    if len(batch_states) >= BATCH_SIZE:
                                        states = np.vstack((states, batch_states))
                                        controls = np.vstack((controls, batch_controls))

                                        save_data(state_file, control_file, states, controls)

                                        batch_states.clear()
                                        batch_controls.clear()
                                        gc.collect()  # Free memory

                                    processed_states += 1

            except Exception as e:
                print(f"Error at state index {processed_states}: {e}")

# Save remaining batch
if batch_states:
    states = np.vstack((states, batch_states))
    controls = np.vstack((controls, batch_controls))
    save_data(state_file, control_file, states, controls)
