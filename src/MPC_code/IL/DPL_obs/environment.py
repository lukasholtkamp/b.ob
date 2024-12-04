import numpy as np

# Utility functions
def gamma(eta):
    if eta == 0:
        return 1
    else:
        return 0.5 * (np.sqrt(1 + (2 * eta) ** 2) + ((np.arcsinh(2 * eta)) / (2 * eta)))

def g(v_max, eta):
    return v_max / gamma(eta)

# Function to generate a parabolic path given η
def f(eta, s, v_max=0.1):
    g_eta = g(v_max, eta)
    x = g_eta * s
    y = eta * x ** 2
    return x, y

# Environment class
class SimpleEnvironment:
    def __init__(self, v_max=0.1, dt=0.3, max_steps=60, num_obstacles=1, epsilon=0.1, inflation=0.18):
        self.v_max = v_max
        self.dt = dt  # Time step for simulation
        self.max_steps = max_steps  # Maximum number of steps per episode
        self.num_obstacles = num_obstacles
        self.epsilon = epsilon  # Minimum distance for obstacles
        self.inflation = inflation
        self.reset()

    def reset(self):
        """
        Reset the environment and initialize the robot and obstacles.
        """
        self.step_count = 0

        # Randomly sample curvature, progress, and orientation
        
        probability = 0.4  # Probability of generating obstacles (adjust as needed)
        if np.random.rand() < probability:
            # Generate obstacles
            eta = np.random.uniform(0, 3)  # Curvature
        else:
            # No obstacles
            eta = 0  # Curvature

        s = np.random.uniform(-0.4, 0.4)  # Path progress
        orientation = np.random.uniform(0, 2 * np.pi)  # Orientation

        # Initial position on the path
        tangent_offset = np.random.uniform(-0.7, 0.7)
        x, y = f(eta, s + tangent_offset, self.v_max)

        # Compute the tangent vector
        g_eta = g(self.v_max, eta)
        tangent_x = g_eta
        tangent_y = 2 * eta * g_eta * (s + tangent_offset)
        tangent_norm = np.sqrt(tangent_x**2 + tangent_y**2)
        tangent_x /= tangent_norm
        tangent_y /= tangent_norm

        # Compute the normal vector and apply a random normal offset
        normal_x = -tangent_y
        normal_y = tangent_x
        normal_offset = np.random.uniform(-1.0, 1.0)
        x += normal_offset * normal_x
        y += normal_offset * normal_y

        probability = 0.5  # Probability of generating obstacles (adjust as needed)
        if np.random.rand() < probability:
            # Generate obstacles
            self.obstacles = self.generate_obstacles(x, y)
        else:
            # No obstacles
            self.obstacles = np.array([[0, 0, 0]])  # Default empty obstacle

        # Initialize the state as an array
        obs_features = self.get_obstacle_features()

        self.state = np.array([x, y, orientation, s, eta] + obs_features.tolist())
        return self.state

    def generate_obstacles(self, x, y):
        """
        Generate obstacles around the robot position.
        """
        obstacles = []
        for _ in range(self.num_obstacles):
            obs_r = np.random.uniform(0.08, 0.2) + self.inflation  # Obstacle radius
            min_distance = obs_r + self.epsilon
            max_distance = min_distance + 0.18

            # Random angle in the specified ranges (to the right of the robot)
            angle = np.random.choice([
                np.random.uniform(0, np.pi / 2),
                np.random.uniform(3 * np.pi / 2, 2 * np.pi)
            ])

            # Compute obstacle position
            distance = np.random.uniform(min_distance, max_distance)
            obs_x = x + distance * np.cos(angle)
            obs_y = y + distance * np.sin(angle)
            obstacles.append([obs_x, obs_y, obs_r])  # Store as list for consistency
        return np.array(obstacles)  # Return as an array

    def get_obstacle_features(self):
        """
        Flatten obstacle features [x1, y1, r1, x2, y2, r2, ...].
        """
        return self.obstacles.flatten()  # Convert 2D array to 1D array

    def dynamics(self, state, action):
        """
        Compute the time derivative of the state.
        """
        x, y, orientation, s, eta = state[:5]
        linear_velocity, angular_velocity, s_dot = action

        dx = linear_velocity * np.cos(orientation)
        dy = linear_velocity * np.sin(orientation)
        d_orientation = angular_velocity
        d_s = s_dot
        d_eta = 0  # Assuming constant curvature

        return np.array([dx, dy, d_orientation, d_s, d_eta])

    def rk4_step(self, state, action):
        """
        Perform a single RK4 integration step.
        """
        dt = self.dt

        # Compute RK4 coefficients
        k1 = self.dynamics(state, action)
        k2 = self.dynamics(state[:5] + 0.5 * dt * k1, action)
        k3 = self.dynamics(state[:5] + 0.5 * dt * k2, action)
        k4 = self.dynamics(state[:5] + dt * k3, action)

        # Update state
        next_state = state[:5] + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        obs_features = state[5:]  # Obstacles remain static
        return np.concatenate([next_state, obs_features])

    def step(self, action):
        """
        Simulate a step in the environment.
        """
        # Perform RK4 integration
        next_state = self.rk4_step(self.state, action)

        # Update state and step count
        self.state = next_state
        self.step_count += 1

        # Done condition
        done = self.step_count >= self.max_steps

        # Reward function (e.g., penalize proximity to origin and obstacles)
        reward = self.compute_reward(self.state, action)

        return self.state, reward, done

    def compute_reward(self, state, action):
        """
        Compute the reward based on path-following and obstacle avoidance.
        """
        robot_position = np.array(state[:2])
        reward = -np.linalg.norm(robot_position)  # Encourage staying near the origin

        # Penalize proximity or collision with obstacles
        for obs in self.obstacles:
            obs_position = np.array(obs[:2])
            obs_r = obs[2]
            distance_to_obs = np.linalg.norm(robot_position - obs_position) - obs_r

            if distance_to_obs < 0:  # Collision
                reward -= 100
            elif distance_to_obs < 0.5:  # Close proximity
                reward -= 10 / (distance_to_obs + 1e-2)

        return reward
