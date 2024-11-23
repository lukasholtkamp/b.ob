import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mcolors
from casadi import SX, vertcat, nlpsol, reshape, if_else, Function, sqrt, arcsinh, cos, sin, bilin, fmax

# Updated gamma function that handles symbolic expressions
def gamma(eta):
    return 0.5 * (sqrt(1 + (2 * eta) ** 2) + arcsinh(2 * eta) / (2 * fmax(eta,10**-10)))

# Updated g function
def g(v_max, eta):
    return if_else(eta == 0, v_max, v_max / gamma(eta))

# The modified reference_traj function
def reference_traj(s, eta=0.5, v_max=1, lbs=-0.2, ubs=0.2):
    result = SX.zeros(3)
    x = g(v_max, eta) * s  # x component
    
    # Parabolic form for x between lbs and ubs
    y_parabola = eta * x**2
    
    # Tangent line equations for regions outside of lbs and ubs
    lbx = lbs
    lby = eta * lbx**2
    mlb = 2 * eta * lbx  # Derivative at lbs (slope of the tangent)
    y_left = mlb * (x - lbx) + lby  # Equation of the tangent line for x < lbs

    rbx = ubs
    rby = eta * rbx**2
    mrb = 2 * eta * rbx  # Derivative at ubs (slope of the tangent)
    y_right = mrb * (x - rbx) + rby  # Equation of the tangent line for x > ubs

    # Combine conditions using if_else
    result[0] = x  # x component
    result[1] = if_else(x < lbs, y_left, if_else(x > ubs, y_right, y_parabola))  # y component

    return result

def mobile_robot_ode(x, u):

    # Define the ODEs
    dx1 = u[0]*cos(x[2])
    dx2 = u[0]*sin(x[2])
    dx3 = u[1]

    dx = vertcat(dx1, dx2, dx3)
    return dx

def rk4(ode, h, x, u):
    """
    Perform the Runge-Kutta 4th order integration.

    Parameters:
    - ode: function that computes the derivative of the state
    - h: time step
    - t: current time (not used in this implementation but kept for consistency)
    - x: current state
    - u: control input

    Returns:
    - xf: next state after integration
    """
    # Calculate the four RK4 coefficients
    k1 = ode( x, u)
    k2 = ode( x + h/2 * k1, u)
    k3 = ode( x + h/2 * k2, u)
    k4 = ode( x + h * k3, u)
    
    # Compute the next state using the RK4 formula
    xf = x + h/6 * (k1 + 2*k2 + 2*k3 + k4)
    return xf

def inequality_constraints(U, W, N):

    lb_u = np.array([0, -0.7])  # Lower bounds on inputs
    ub_u = np.array([1, 0.7])   # Upper bounds on inputs
    lb_w = 0
    ub_w = 1
   
    hu = []   # Box constraints on input
    hs = []   # Box constraints on s
    hw = []
 
    for i in range(N):
        hu.append(lb_u - U[:, i]) 
        hu.append(U[:, i] - ub_u)
        hw.append(lb_w - W[i])
        hw.append(W[i]-ub_w)

    return  hu, hs, hw
# Define the objective cost function outside of run_open_loop_mpc
def objective_cost(X, U, S_a, W, N, reference_trajectory, dyn_cost_fcn, Q):
    J = 0.0
    for i in range(N):
        dx = X[:, i] - reference_trajectory(S_a[i])
        du = U[:, i]
        J += dyn_cost_fcn(dx, du, W[i])
    J += bilin(Q, X[:, N] - reference_trajectory(S_a[N]))
    return J

def objective_cost(X, U, S_a, W, N, reference_trajectory, dyn_cost_fcn, Q, obs_list, mu=8 * 10**2):
    J = 0.0
    for i in range(N):
        dx = X[:, i] - reference_trajectory(S_a[i])
        du = U[:, i]
        J += dyn_cost_fcn(dx, du, W[i])

        # Obstacle avoidance term
        for obs in obs_list:
            obs_x, obs_y, obs_radius = obs
            h = fmax((obs_radius + 0.23)**2 - (X[0, i] - obs_x)**2 - (X[1, i] - obs_y)**2, 0)
            J += 0.5 * mu * h**2  # Add weighted obstacle cost

    # Terminal cost with obstacles
    J += bilin(Q, X[:, N] - reference_trajectory(S_a[N]))
    for obs in obs_list:
        obs_x, obs_y, obs_radius = obs
        h_e = fmax((obs_radius + 0.23)**2 - (X[0, N] - obs_x)**2 - (X[1, N] - obs_y)**2, 0)
        J += 0.5* mu * h_e**2  # Add weighted terminal obstacle cost

    return J


# Define the equality constraints function outside of run_open_loop_mpc
def equality_constraints(X, U, S_a, W, P_a, Ts, system, N, nx):
    g = []  # Equality constraints initialization
    g.append(X[:, 0] - P_a[:nx])  # Initial state constraint
    g.append(S_a[0]-P_a[nx:nx+1])
    for i in range(N):
        st = X[:, i]
        cons = U[:, i] 
        st_next_euler = rk4(system,Ts, st, cons)
        s_next_euler = S_a[i]+Ts*W[i]
        st_next = X[:, i+1]
        s_next = S_a[i+1] 
        g.append(st_next -  st_next_euler)
        g.append(s_next - s_next_euler )
    return g

# The main function that uses the above-defined functions
def run_open_loop_mpc(v_max, x0, s0, eta, obs_list):
    # Controller frequency and prediction horizon
    Ts = 0.3
    N = 60
    nx = 3
    nu = 2
    T = 10
    mu = 8 * 10**2
    
    x = SX.sym('x', nx)
    u = SX.sym('u', nu)
    s = SX.sym('s', 1)
    w = SX.sym('w', 1)
    
    dx = mobile_robot_ode(x, u)
    ref = reference_traj(s, eta, v_max)
    
    reference_trajectory = Function("ref_traj", [s], [ref])
    system = Function("sys", [x, u], [dx])
    
    U = SX.sym('U', nu, N)
    X = SX.sym('X', nx, N + 1)
    S_a = SX.sym('S_a', N + 1, 1)
    W = SX.sym('W', 1, N)
    
    P_a = SX.sym('P_a', nx + 1)
    
    Q = np.diag([10, 10, 0])
    R = np.diag([1, 1])
    
    V_dyn = bilin(Q, x) + bilin(R, u) + bilin(T, (1 - w))
    dyn_cost_fcn = Function("dyn_cost", [x, u, w], [V_dyn])
    
    # Call the objective cost function
    J = objective_cost(X, U, S_a, W, N, reference_trajectory, dyn_cost_fcn, Q,obs_list,mu)
    
    g = equality_constraints(X, U, S_a, W, P_a, Ts, system, N, nx)
    G = vertcat(*g)
    hu, hs, hw = inequality_constraints(U, W, N)
    Hs = vertcat(*hs)
    Hu = vertcat(*hu)
    Hw = vertcat(*hw)
    G_vcsd = vertcat(*g, *hu, *hw, *hs)
    
    # Set the bounds for the constraints
    lbg = [0] * G.shape[0] + [-np.inf] * (Hu.shape[0] + Hs.shape[0] + Hw.shape[0])
    ubg = [0] * G.shape[0] + [0] * (Hu.shape[0] + Hs.shape[0] + Hw.shape[0])
    lbg_vcsd = vertcat(*lbg)
    ubg_vcsd = vertcat(*ubg)
    
    opts_setting = {
        "ipopt.max_iter": 500,
        "ipopt.print_level": 0,  # Set print level to show detailed output
        "print_time": True,      # Print total time information
        "ipopt.sb": "yes",       # Print summary banner
        "ipopt.tol": 1e-6,
        "ipopt.acceptable_tol": 1e-6,
        "ipopt.acceptable_obj_change_tol": 1e-6,
    }
    
    prob = {
        "f": J,
        "x": vertcat(reshape(X, -1, 1), reshape(U, -1, 1), reshape(W, -1, 1), reshape(S_a, -1, 1)),
        "p": vertcat(P_a),
        "g": G_vcsd
    }
    solver = nlpsol("solver", "ipopt", prob, opts_setting)
    
    x_st_0 = np.tile(x0, (N + 1, 1)).T
    u_st_0 = np.zeros((N, nu))
    w_st_0 = np.ones((N, 1))
    s_st_0 = np.tile(s0, (N + 1, 1))
    
    args_x0 = np.concatenate([
        x_st_0.T.reshape(-1),
        u_st_0.T.reshape(-1),
        w_st_0.T.reshape(-1),
        s_st_0.T.reshape(-1)
    ])
    
    sol = solver(x0=args_x0, p=np.concatenate([x0, [s0]]), lbg=lbg, ubg=ubg)

    # Extract the solution for states X
    x_pred = np.array(sol['x'][:nx * (N + 1)]).reshape((N + 1, nx))
    
    # Extract the solution for controls U
    u_start_idx = nx * (N + 1)
    u_end_idx = u_start_idx + nu * N
    u = np.array(sol['x'][u_start_idx:u_end_idx]).reshape((N, nu))
    
    # Extract the solution for W
    w_start_idx = u_end_idx
    w_end_idx = w_start_idx + N
    w = np.array(sol['x'][w_start_idx:w_end_idx])

    u = np.append(u,w,axis=1)
    
    return x_pred, u

# x0 = np.array([-0.5, -0.1, -np.pi/4])  # Initial state [x, y, theta]
# s0 = 1  # Initial reference position
# v_max = 1
# eta_val = 0

# # Define the obstacles (example: [x, y, radius])
# obs_list = [[0.0, 0.0, 0.17+0.18]]  # You can modify or add more obstacles


# x_pred, u = run_open_loop_mpc(v_max, x0, s0, eta_val, obs_list=obs_list)

# # Create CasADi function for reference trajectory

# s = SX.sym('s')

# eta = SX.sym('eta')

# v_max = 1
# dk = 0.5
# reference_traj_output = reference_traj(s, eta=eta, v_max=v_max)
# reference_traj_func = Function('ref_traj', [s, eta], [reference_traj_output[0], reference_traj_output[1]])

# # Generate s values and compute the trajectory
# g_eta = g(v_max, eta_val)
# s_values = np.linspace(-dk / g_eta, dk / g_eta, 100).flatten()

# x_values = [reference_traj_func(s_val, eta_val)[0].toarray().item() for s_val in s_values]
# y_values = [reference_traj_func(s_val, eta_val)[1].toarray().item() for s_val in s_values]

# # Plot the reference trajectory

# plt.figure(figsize=(10, 6))

# # Plot obstacles
# for obs in obs_list:
#     obs_x, obs_y, obs_radius = obs
#     obstacle_circle = plt.Circle((obs_x, obs_y), obs_radius, color='orange', alpha=0.5, label='Obstacle' if obs == obs_list[0] else "")
#     plt.gca().add_patch(obstacle_circle)

# # Calculate the coordinates for s0 on the reference trajectory
# s0_x, s0_y = reference_traj_func(s0, eta_val)

# # Convert CasADi results to scalar values
# s0_x = s0_x.toarray().item()
# s0_y = s0_y.toarray().item()

# # Plot the reference trajectory
# plt.plot(x_values, y_values, label='Reference Trajectory', color='blue')

# # Plot the x0 point
# plt.plot(x0[0], x0[1], 'ro', label='Initial Point (x0)')

# # Plot the s0 point
# plt.plot(s0_x, s0_y, 'go', label='s0 Point', markersize=8)  # s0 point in green

# # Add an arrow for orientation
# arrow_length = 0.1  # Length of the arrow
# plt.arrow(x0[0], x0[1], arrow_length * np.cos(x0[2]), arrow_length * np.sin(x0[2]),
#           head_width=0.02, head_length=0.04, fc='red', ec='red', label='Orientation')

# trajectory_line, = plt.plot(x_pred[:, 0], x_pred[:, 1], label='Open Loop Trajectory', color='red')

# # Plot arrows along the trajectory

# length = 0.1  # Length of the arrows

# u_arrows = length * np.cos(x_pred[:, 2])

# v_arrows = length * np.sin(x_pred[:, 2])

# # Generate a colormap from green to red

# norm = mcolors.Normalize(vmin=0, vmax=len(x_pred) - 1)

# cmap = cm.get_cmap('RdYlGn_r', len(x_pred))


# arrow_quivers = []

# for i in range(len(x_pred)):

#     color = cmap(norm(i))

#     quiver = plt.quiver(

#         x_pred[i, 0], x_pred[i, 1],

#         u_arrows[i], v_arrows[i],

#         angles='xy', scale_units='xy', scale=1, color=color

#     )

#     arrow_quivers.append(quiver)




# # Add labels and grid
# plt.title('Reference Trajectory with Initial Point, Orientation, and s0 Point')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.grid(True)
# plt.axis('equal')
# plt.xlim(-1, 1)
# plt.ylim(-1, 1)
# plt.legend()

# plt.show()
