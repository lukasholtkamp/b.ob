import numpy as np
import os
import json
import torch
import matplotlib.pyplot as plt
from paths.path_segments_save import load_path_segments, PathSegment
from environment import SimpleEnvironment
from transform import T_z_obs,error
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
    def __init__(self, policy_path, environment, path_segments_file):
        self.max_obstacles = 1  # Single obstacle

        # Load path segments
        self.path_segments = load_path_segments(path_segments_file)

        # Define f_s using CasADi symbolic evaluation
        s = ca.MX.sym("s")
        selected_result = self.f(self.path_segments, s)
        self.f_s = ca.Function("f_s", [s], [selected_result])

        # Initialize policy model with input size based on a single obstacle
        input_dim = 5 + self.max_obstacles * 3  # 5 for state, 3 per obstacle
        self.policy = PolicyModel(input_dim=input_dim, output_dim=3)
        self.policy.load_state_dict(torch.load(policy_path))
        self.policy.eval()

        self.env = environment
        self.closed_loop_trajectory = []

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

    def run(self, max_iterations=1000, tolerance=0.2):
        state = self.env.reset()
        done = False
        self.closed_loop_trajectory = []
        iteration = 0

        # Compute goal position from reference path
        self.goal_position = (
            self.f_s(self.path_segments[-1].end_time).toarray().flatten()[:2]
        )

        while not done and iteration < max_iterations:
            iteration += 1

            # Transform state into path-following frame with obstacles
            transformed_state = T_z_obs(
                self.path_segments,
                state[:5],  # x, y, theta, s, eta
                state[5:],  # Obstacles
            )

            # Use the trained policy to decide the next action
            state_tensor = torch.tensor(transformed_state, dtype=torch.float32)
            action = self.policy(state_tensor).detach().numpy()

            # Save trajectory
            self.closed_loop_trajectory.append(state[:3])  # x, y, theta

            # Take a step in the environment
            state, _, done = self.env.step(action)

            # Check if within tolerance of the goal
            position = np.array(state[:2])  # x, y
            if np.linalg.norm(position - self.goal_position) <= tolerance:
                logging.info("Goal reached within tolerance.")
                break

            if iteration % 100 == 0:
                logging.info(f"Iteration {iteration}: State = {state}")

        if iteration >= max_iterations:
            logging.warning("Max iterations reached before reaching the goal.")

    def plot_results(self):
        """Plot the reference path, closed-loop trajectory, and obstacles."""
        plt.figure(figsize=(10, 10))

        # Plot the single obstacle
        obstacle = self.env.obstacles[0]
        circle = plt.Circle(
            (obstacle[0], obstacle[1]), obstacle[2], color="r", alpha=0.5
        )
        plt.gca().add_patch(circle)
        plt.scatter(
            obstacle[0],
            obstacle[1],
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
    model_path = os.path.join(MODELS_DIR, "final_policy.pth")
    path_segments_file = os.path.join("paths", "path_segments_left.json")

    # Initialize the environment
    env = SimpleEnvironment(
        dt=0.3,
        max_steps=60,
        num_obstacles=1,  # Single obstacle
    )

    # Initialize the controller
    controller = ObstacleAvoidanceController(
        policy_path=model_path,
        environment=env,
        path_segments_file=path_segments_file,
    )

    # Run the simulation
    controller.run(max_iterations=1000, tolerance=0.2)

    # Plot the results
    controller.plot_results()
