import numpy as np
from NMPC_solver import *

def expert_policy(state,v_max=0.1):
    """
    Placeholder expert policy.
    Args:
        state: [x, y, orientation, s, eta]
    Returns:
        action: [linear_velocity, angular_velocity, s_dot]
    """
    x, y, orientation, s, eta = state
    _,u = run_open_loop_mpc(v_max, [x,y,orientation], s,eta)

    return np.array([u[0,0],u[0,1],u[0,2]])
