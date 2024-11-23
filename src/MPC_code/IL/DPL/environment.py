import numpy as np

def gamma(eta):
    if eta==0:
        return 1
    else:
        return 0.5*(np.sqrt(1+(2*eta)**2)+((np.arcsinh(2*eta))/(2*eta)))

def g(v_max,eta):
    return v_max/gamma(eta)

# Function to generate a parabolic path given η
def f(eta,s,v_max=0.1):
    g_eta = g(v_max,eta)
    x = g_eta * s
    y = eta * x ** 2
    return x, y

class SimpleEnvironment:
    def __init__(self, v_max=0.1, dt=0.3, max_steps=60):
        self.v_max = v_max
        self.dt = dt  # Time step for simulation
        self.max_steps = max_steps  # Maximum number of steps per episode
        self.reset()

    def reset(self):
        # Reset the environment and step counter
        self.step_count = 0

        # Randomly sample curvature, progress, and orientation
        eta = np.random.uniform(0, 3)  # Curvature
        s = np.random.uniform(-0.4, 0.4)  # Path progress
        orientation = np.random.uniform(0, 2 * np.pi)  # Orientation

        # Initial position on the path
        tangent_offset = np.random.uniform(-0.7, 0.7)
        x, y = f(eta, s + tangent_offset, self.v_max)

        # Compute the tangent vector at the adjusted (x, y)
        g_eta = g(self.v_max, eta)
        tangent_x = g_eta  # dx/ds
        tangent_y = 2 * eta * g_eta * (s + tangent_offset)  # dy/ds
        tangent_norm = np.sqrt(tangent_x**2 + tangent_y**2)
        tangent_x /= tangent_norm
        tangent_y /= tangent_norm

        # Compute normal vector and apply normal offset
        normal_x = -tangent_y
        normal_y = tangent_x
        normal_offset = np.random.uniform(-1.0, 1.0)
        x += normal_offset * normal_x
        y += normal_offset * normal_y

        # Initialize the state
        self.state = np.array([x, y, orientation, s, eta])
        return self.state

    def dynamics(self, state, action):
        """
        Compute the time derivative of the state.
        Args:
            state: Current state [x, y, orientation, s, eta].
            action: Control inputs [linear_velocity, angular_velocity, s_dot].
        Returns:
            dx/dt: Derivatives of the state.
        """
        x, y, orientation, s, eta = state
        linear_velocity, angular_velocity, s_dot = action

        dx = linear_velocity * np.cos(orientation)
        dy = linear_velocity * np.sin(orientation)
        d_orientation = angular_velocity
        d_s = s_dot
        d_eta = 0  # Assuming curvature (eta) is constant

        return np.array([dx, dy, d_orientation, d_s, d_eta])

    def rk4_step(self, state, action):
        """
        Perform a single RK4 integration step.
        Args:
            state: Current state [x, y, orientation, s, eta].
            action: Control inputs [linear_velocity, angular_velocity, s_dot].
        Returns:
            next_state: Updated state after RK4 integration.
        """
        dt = self.dt

        # Compute RK4 coefficients
        k1 = self.dynamics(state, action)
        k2 = self.dynamics(state + 0.5 * dt * k1, action)
        k3 = self.dynamics(state + 0.5 * dt * k2, action)
        k4 = self.dynamics(state + dt * k3, action)

        # Update state
        next_state = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return next_state

    def step(self, action):
        """
        Simulates a step in the environment using RK4.
        Args:
            action: [linear_velocity, angular_velocity, s_dot]
        Returns:
            next_state: Updated state after applying action.
            reward: Reward for evaluation purposes.
            done: Boolean indicating whether the episode is over.
        """
        # Perform RK4 integration
        next_state = self.rk4_step(self.state, action)

        # Update state and step count
        self.state = next_state
        self.step_count += 1

        # Done condition: Episode ends after max_steps
        done = self.step_count >= self.max_steps

        # Reward (optional, for evaluation purposes)
        reward = -np.linalg.norm(self.state[:2])  # Negative distance from origin

        return self.state, reward, done
