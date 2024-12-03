import numpy as np
from NMPC_solver_oa import *

def expert_policy(state, v_max=0.1):
    """
    Expert policy considering obstacles.
    Args:
        state: [x, y, orientation, s, eta, x_obs1, y_obs1, r_obs1, ...]
    Returns:
        action: [linear_velocity, angular_velocity, s_dot]
    """
    x, y, orientation, s, eta = state[:5]
    obstacle_features = state[5:]  # [x_obs1, y_obs1, r_obs1, ...]
    obstacles = [
        (obstacle_features[i], obstacle_features[i + 1], obstacle_features[i + 2])
        for i in range(0, len(obstacle_features), 3)
    ]
    _, u = run_open_loop_mpc(v_max, [x, y, orientation], s, eta, obstacles)
    return np.array([u[0, 0], u[0, 1], u[0, 2]])
