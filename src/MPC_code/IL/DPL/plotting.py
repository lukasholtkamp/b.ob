import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
from paths.path_segments_save import load_path_segments, PathSegment
from paths.transform import T_z
import torch
from policy_model import PolicyModel

from matplotlib import rcParams
# Configure PGF for LaTeX export
rcParams.update({
    "pgf.texsystem": "pdflatex",  # Use pdflatex or xelatex
    "text.usetex": True,          # Enable LaTeX text rendering
    "font.family": "serif",       # Match LaTeX document fonts
    "pgf.preamble": [
        r"\usepackage{amsmath}",  # Use additional LaTeX packages if needed
    ],
    'font.size': 18
})


class PathFollowingController:
    def __init__(
        self, model_path, path_segments_file, dt=0.3, max_steps=2500, tolerance=0.1
    ):
        self.path_segments = load_path_segments(path_segments_file)
        self.dt = dt
        self.max_steps = max_steps
        self.tolerance = tolerance  # Tolerance for reaching the goal position

        # Define f_s using CasADi symbolic evaluation
        s = ca.MX.sym("s")
        selected_result = self.f(self.path_segments, s)
        self.f_s = ca.Function("f_s", [s], [selected_result])

        # Load the trained policy model
        self.model = PolicyModel(input_dim=5, output_dim=3)
        state_dict = torch.load(model_path)
        self.model.load_state_dict(state_dict)
        self.model.eval()

        # Set initial state: [x, y, theta, s]
        initial_point = self.f_s(0).full().flatten()[:2]
        self.current_state = np.array(
            [initial_point[0], initial_point[1], 0, 0]
        )  # Assume theta = 0
        self.closed_loop_trajectory = []

        # Set the goal position (final point on the path)
        final_point = self.f_s(self.path_segments[-1].end_time).full().flatten()[:2]
        self.goal_position = np.array(final_point)

    def f(self, segments, s):
        """Select the correct segment for a given path parameter s."""
        result = ca.MX.zeros(3)  # Initialize CasADi variable for (x, y, z)
        for segment in segments:
            condition = ca.logic_and(s >= segment.start_time, s <= segment.end_time)
            result = ca.if_else(condition, segment.f(s), result)
        # Handle s values beyond the last segment's end_time
        last_segment = segments[-1]
        result = ca.if_else(
            s > last_segment.end_time, last_segment.f(last_segment.end_time), result
        )
        return result

    def simulate_policy_until_goal(self):
        iteration = 0
        while (
            np.linalg.norm(self.current_state[:2] - self.goal_position) > self.tolerance
            and iteration < self.max_steps
        ):
            # Transform current state
            x_hat, y_hat, theta_hat, s_hat, eta = T_z(
                self.path_segments,
                self.current_state[0],
                self.current_state[1],
                self.current_state[2],
                self.current_state[3],
            )

            # Prepare input
            if eta >= 0:
                model_input = np.array([[x_hat, y_hat, theta_hat, s_hat, eta]])
            else:
                model_input = np.array([[x_hat, -y_hat, -theta_hat, s_hat, -eta]])

            # Predict
            model_input_tensor = torch.tensor(model_input, dtype=torch.float32)
            usol = self.model(model_input_tensor).detach().numpy()

            # Adjust if eta < 0
            if eta < 0:
                usol[0][1] *= -1

            # Clip controls
            usol[0][0] = np.clip(usol[0][0], 0, 1)
            usol[0][1] = np.clip(usol[0][1], -0.8, 0.8)
            usol[0][2] = np.clip(usol[0][2], 0, 1)

            # Simulate step
            self.current_state = self.simulate_kinematic_step(self.current_state, usol[0])

            # Store full state now, including s
            # current_state: [x, y, theta, s]
            self.closed_loop_trajectory.append(self.current_state.copy())

            iteration += 1

        if iteration >= self.max_steps:
            print("Simulation stopped: Maximum number of iterations reached.")
        else:
            print("Simulation stopped: Goal reached within tolerance.")


    def simulate_kinematic_step(self, state, control):
        """Simulate the next state of the system using kinematic equations."""
        x, y, theta, s = state
        v, omega, s_dot = control
        x_next = x + self.dt * v * np.cos(theta)
        y_next = y + self.dt * v * np.sin(theta)
        theta_next = theta + self.dt * omega
        s_next = s + self.dt * s_dot
        return np.array([x_next, y_next, theta_next, s_next])

    def plot_results(self):
        """Plot the reference path and closed-loop trajectory."""
        # Generate reference path
        s_values = np.linspace(0, self.path_segments[-1].end_time, 500)
        ref_path = np.array(
            [self.f_s(s_val).full().flatten()[:2] for s_val in s_values]
        )

        plt.figure(figsize=(10, 10))

        # Plot the reference path
        plt.plot(ref_path[:, 0], ref_path[:, 1], "g--", label="Reference Path")

        # Plot the closed-loop trajectory
        closed_loop = np.array(self.closed_loop_trajectory)
        plt.plot(
            closed_loop[:, 0], closed_loop[:, 1], "b-", label="Closed-Loop Trajectory"
        )

        # Mark start point
        plt.scatter(
            self.closed_loop_trajectory[0][0],
            self.closed_loop_trajectory[0][1],
            color="black",
            label="Start Point",
        )

        # Mark goal position
        plt.scatter(
            self.goal_position[0],
            self.goal_position[1],
            color="red",
            label="Goal Position",
        )

        # Plot settings
        plt.title("Reference Path and Closed-Loop Trajectory")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.show()


if __name__ == "__main__":
    # Define the model paths for iterations 1, 4, 7, and 10
    model_paths = [
        "models/policy_iteration_1.pth",
        "models/policy_iteration_4.pth",
        "models/policy_iteration_7.pth",
        "models/policy_iteration_10.pth",
    ]
    path_segments_file = "paths/path_segments_left.json"

    # Colors for each model's trajectory
    colors = ["blue", "orange", "purple", "cyan"]
    labels = ["Iteration 1", "Iteration 4", "Iteration 7", "Iteration 10"]

    # Create a plot
    plt.figure(figsize=(10, 10))

    # Generate reference path
    controller = PathFollowingController(model_paths[0], path_segments_file)
    s_values = np.linspace(0, controller.path_segments[-1].end_time, 500)
    ref_path = np.array([controller.f_s(s_val).full().flatten()[:2] for s_val in s_values])

    # Plot the reference path
    plt.plot(ref_path[:, 0], ref_path[:, 1], "g-", label="Reference Path")

    # Simulate and plot each model's trajectory
    for model_path, color, label in zip(model_paths, colors, labels):
        controller = PathFollowingController(model_path, path_segments_file)
        controller.simulate_policy_until_goal()
        closed_loop = np.array(controller.closed_loop_trajectory)

        # Plot the trajectory
        plt.plot(closed_loop[:, 0], closed_loop[:, 1], "--", color=color, label=label)

        # After plotting the reference path
        start_x, start_y = ref_path[0, 0], ref_path[0, 1]
        end_x, end_y = ref_path[-1, 0], ref_path[-1, 1]

        # Plot markers for start and end
        plt.plot(start_x, start_y, 'bo')  # Green point at start
        plt.plot(end_x, end_y, 'ro')      # Red point at end

        # Add text labels underneath both points
        offset = 0.2  # Adjust as needed for desired spacing
        plt.text(
            start_x, start_y - offset, "Start",
            fontsize=18, color='black', horizontalalignment='center', verticalalignment='top',
            bbox=dict(facecolor='white', alpha=0.8, edgecolor='none')
        )
        plt.text(
            end_x, end_y - offset, "End",
            fontsize=18, color='black', horizontalalignment='center', verticalalignment='top',
            bbox=dict(facecolor='white', alpha=0.8, edgecolor='none')
        )
    

    # Plot settings
    # plt.title("Progression of Policy Iterations")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")

    plt.savefig("DAgger_itter_pf.pgf", bbox_inches='tight')  # Export to PGF for LaTeX integration


    # plt.show()
        # Generate paths for iterations 1 through 10
    iterations = range(1, 11)
    model_paths = [f"models/policy_iteration_{i}.pth" for i in iterations]

    MSE_values = []

    for i, model_path in zip(iterations, model_paths):
        controller = PathFollowingController(model_path, path_segments_file)
        controller.simulate_policy_until_goal()
        closed_loop = np.array(controller.closed_loop_trajectory)  # Nx4: x, y, theta, s

        # Compute MSE
        errors = []
        for state in closed_loop:
            x, y, theta, s_val = state
            ref_point = controller.f_s(s_val).full().flatten()[:2]
            ref_x, ref_y = ref_point
            error_sq = (x - ref_x)**2 + (y - ref_y)**2
            errors.append(error_sq)

        mse = np.mean(errors)
        MSE_values.append(mse)
        print(f"Iteration {i}: MSE = {mse}")

    # Plot MSE vs Iteration on a log scale
    plt.figure(figsize=(8, 6))
    plt.semilogy(iterations, MSE_values, marker='o', linestyle='--', color='blue', label='MSE')
    plt.xlabel("Iteration")
    plt.ylabel(r"log$_{10}$(MSE)")
    # plt.title("MSE Over Iterations (1 through 10) - Log Scale")
    # plt.grid(True, which="both", linestyle='--')

    # Save the MSE plot as PGF
    plt.savefig("MSE_iterations.pgf", bbox_inches='tight')
    # plt.show()
