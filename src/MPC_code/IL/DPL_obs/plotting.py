import numpy as np
import os
import json
import torch
import matplotlib.pyplot as plt
from paths.path_segments_save import load_path_segments, PathSegment
from environment import SimpleEnvironment
from paths.transform import T_z_obs,error, get_deviated_point
from policy_model import PolicyModel
import logging
import casadi as ca

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
)

# Paths for data and models
DATA_DIR = "data/"
MODELS_DIR = "models/"
TRAJECTORIES_DIR = "trajectories/"
os.makedirs(MODELS_DIR, exist_ok=True)
os.makedirs(TRAJECTORIES_DIR, exist_ok=True)


class ObstacleAvoidanceController:
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
        self.model = PolicyModel(obstacle_count=1)
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

    def simulate_policy_with_obstacles(self, obstacle_s=15.0, obstacle_deviation=0.5):
        """
        Simulate the policy with obstacle avoidance until within tolerance of the goal position.
        """
        # Define the obstacle along the path
        self.obs_position = get_deviated_point(self.path_segments, obstacle_s, obstacle_deviation)
        self.obs_r = 0.2  # Example obstacle radius
        self.obs_inflation = 0.2  # Safety margin around the obstacle

        # Reset trajectory and simulation parameters
        self.closed_loop_trajectory = []
        iteration = 0

        while (
            np.linalg.norm(self.current_state[:2] - self.goal_position) > self.tolerance
            and iteration < self.max_steps
        ):
            # Transform the current state into the path-following frame with obstacle relevance
            x_hat, y_hat, theta_hat, s_hat, eta, obs_x_hat, obs_y_hat, relevant_flag = T_z_obs(
                self.path_segments,
                self.current_state[0],
                self.current_state[1],
                self.current_state[2],
                self.current_state[3],  # s
                self.obs_position[0],
                self.obs_position[1],
                self.obs_r,
            )

            # Prepare the input for the policy model
            if eta >= 0:
                if relevant_flag:
                    model_input = np.array([[x_hat, y_hat, theta_hat, s_hat, eta, obs_x_hat, obs_y_hat, self.obs_r + self.obs_inflation]])
                else:
                    model_input = np.array([[x_hat, y_hat, theta_hat, s_hat, eta, 0, 0, 0]])
            else:
                if relevant_flag:
                    model_input = np.array([[x_hat, -y_hat, -theta_hat, s_hat, -eta, obs_x_hat, -obs_y_hat, self.obs_r + self.obs_inflation]])
                else:
                    model_input = np.array([[x_hat, -y_hat, -theta_hat, s_hat, -eta, 0, 0, 0]])

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

            # Simulate the next state using kinematic equations
            self.current_state = self.simulate_kinematic_step(
                self.current_state, usol[0]
            )

            # Save the trajectory (x, y, theta)
            self.closed_loop_trajectory.append(self.current_state[:3])
            iteration += 1

            # Log the progress periodically
            if iteration % 100 == 0:
                logging.info(f"Iteration {iteration}: State = {self.current_state}")

        if iteration >= self.max_steps:
            logging.warning("Simulation stopped: Maximum number of iterations reached.")
        else:
            logging.info("Simulation stopped: Goal reached within tolerance.")


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
        """Plot the reference path, closed-loop trajectory, and obstacles."""
        plt.figure(figsize=(10, 10))

        # Plot the single obstacl
        circle = plt.Circle(
            (self.obs_position[0], self.obs_position[1]), self.obs_r, color="r", alpha=0.5
        )
        plt.gca().add_patch(circle)
        plt.scatter(
            self.obs_position[0],
            self.obs_position[1],
            color="red",
            label="Obstacle",
        )

        # Plot the reference path
        s_values = np.linspace(0, self.path_segments[-1].end_time, 500)
        ref_path = np.array([self.f_s(s).toarray().flatten()[:2] for s in s_values])
        plt.plot(ref_path[:, 0], ref_path[:, 1], "g--", label="Reference Path")

        # Plot the closed-loop trajectory
        closed_loop = np.array(self.closed_loop_trajectory)
        plt.plot(
            closed_loop[:, 0],
            closed_loop[:, 1],
            "b-",
            label="Closed-Loop Trajectory",
        )

        # Mark start and goal positions
        start_pos = self.closed_loop_trajectory[0][:2]
        plt.scatter(start_pos[0], start_pos[1], color="black", label="Start Position")
        plt.scatter(
            self.goal_position[0],
            self.goal_position[1],
            color="green",
            label="Goal Position",
        )

        # Plot settings
        plt.title("Path Following with Single Obstacle Avoidance")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.show()


if __name__ == "__main__":
    # File paths
    model_path = os.path.join(MODELS_DIR, "policy_iteration_8.pth")
    path_segments_file = os.path.join("paths", "path_segments_left.json")

    # Initialize the controller
    controller = ObstacleAvoidanceController(
        model_path=model_path,
        path_segments_file=path_segments_file,
    )

    # Simulate the policy
    # controller.simulate_policy_with_obstacles(obstacle_s=55.0, obstacle_deviation=-0.1)
    controller.simulate_policy_with_obstacles(obstacle_s=57.0, obstacle_deviation=-0.1)

    # Plot the results
    controller.plot_results()

