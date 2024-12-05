import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
from paths.path_segments_save import load_path_segments, PathSegment
from paths.transform import T_z
import torch
from policy_model import PolicyModel


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
        """Simulate the policy until within tolerance of the goal position."""
        iteration = 0
        while (
            np.linalg.norm(self.current_state[:2] - self.goal_position) > self.tolerance
            and iteration < self.max_steps
        ):
            # Transform the current state to the path-following frame
            x_hat, y_hat, theta_hat, s_hat, eta = T_z(
                self.path_segments,
                self.current_state[0],
                self.current_state[1],
                self.current_state[2],
                self.current_state[3],
            )

            # Prepare the input for the policy model
            if eta >= 0:
                model_input = np.array([[x_hat, y_hat, theta_hat, s_hat, eta]])
            else:
                model_input = np.array([[x_hat, -y_hat, -theta_hat, s_hat, -eta]])

            # Convert input to PyTorch tensor and predict
            model_input_tensor = torch.tensor(model_input, dtype=torch.float32)
            usol = self.model(model_input_tensor).detach().numpy()

            # Adjust control outputs if eta < 0
            if eta < 0:
                usol[0][1] *= -1

            # Clip the control outputs
            usol[0][0] = np.clip(usol[0][0], 0, 1)
            usol[0][1] = np.clip(usol[0][1], -0.8, 0.8)
            usol[0][2] = np.clip(usol[0][2], 0, 1)

            # Simulate the next state
            self.current_state = self.simulate_kinematic_step(
                self.current_state, usol[0]
            )

            # Save trajectory
            self.closed_loop_trajectory.append(self.current_state[:3])
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
    colors = ["blue", "orange", "purple", "green"]
    labels = ["Iteration 1", "Iteration 4", "Iteration 7", "Iteration 10"]

    # Create a plot
    plt.figure(figsize=(10, 10))

    # Generate reference path
    controller = PathFollowingController(model_paths[0], path_segments_file)
    s_values = np.linspace(0, controller.path_segments[-1].end_time, 500)
    ref_path = np.array([controller.f_s(s_val).full().flatten()[:2] for s_val in s_values])

    # Plot the reference path
    plt.plot(ref_path[:, 0], ref_path[:, 1], "g--", label="Reference Path")

    # Simulate and plot each model's trajectory
    for model_path, color, label in zip(model_paths, colors, labels):
        controller = PathFollowingController(model_path, path_segments_file)
        controller.simulate_policy_until_goal()
        closed_loop = np.array(controller.closed_loop_trajectory)

        # Plot the trajectory
        plt.plot(closed_loop[:, 0], closed_loop[:, 1], "-", color=color, label=label)

    # Mark the goal position
    plt.scatter(
        controller.goal_position[0],
        controller.goal_position[1],
        color="red",
        label="Goal Position",
    )

    # Plot settings
    # plt.title("Progression of Policy Iterations")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()
