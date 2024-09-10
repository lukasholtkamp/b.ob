import numpy as np
from casadi import *
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle


def mobile_robot_ode(x, u):

    # (state space )= (x1=x ,  x2 = y , x3= theta)
    # (input space ) = (u1= v , u2=w)

    # Define the ODEs
    dx1 = u[0] * cos(x[2])
    dx2 = u[0] * sin(x[2])
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
    k1 = mobile_robot_ode(x, u)
    k2 = mobile_robot_ode(x + h / 2 * k1, u)
    k3 = mobile_robot_ode(x + h / 2 * k2, u)
    k4 = mobile_robot_ode(x + h * k3, u)

    # Compute the next state using the RK4 formula
    xf = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return xf


def shift(T, t0, x0, u, f):
    """
    Shift the state and time forward by one timestep.

    Args:
        T (float): The timestep.
        t0 (float): The current time.
        x0 (np.array): The current state.
        u (np.array): The control inputs.
        f (Function): The system function.

    Returns:
        tuple: The updated time, state, and control inputs.
    """

    st = x0
    con = u[0, :]
    # f_value = f(st, con)
    # st = st + T * f_value
    st = rk4(f, T, st, con)
    x0 = np.array(st.full()).flatten()

    t0 = t0 + T
    u0 = np.vstack([u[1:], u[-1, :]])

    return t0, x0, u0


def reference_traj(s):
    T = 90
    etat1 = 6 * cos((2 * pi / T) * s)
    etat2 = 3 * sin((4 * pi / T) * s)
    # etat1 = s
    # etat2 = ((s/40)-2)**2
    eta = vertcat(etat1, etat2, 0)
    return eta


def obstacle1(x, r=1.2):
    h = fmax(r**2 - x[0] ** 2 - x[1] ** 2,0)
    return h

def obstacle(x, obs):
    h = fmax(obs[2]**2 - (x[0]-obs[0]) ** 2 - (x[1]-obs[1]) ** 2,0)
    return h

def obstacle_cost(x_ob, x_ob_a, obs_list, mu=10):

    V_obs = 0

    for i in range(2):

        h_ob = obstacle(x_ob,obs_list[i,:])
        h_ob_a = obstacle(x_ob_a,obs_list[i,:])

        cost = if_else(obs_list[i,2]>0,0.5 * mu * h_ob**2 + 0.5 * mu * h_ob_a**2,0)

        V_obs += cost

    return V_obs


# Controller frequency and Prediction horizon
Ts = 1  # sampling time in [s]

lmda = 0.05

N = int(6/Ts) # prediction horizon

tf = 3

nx = 3  # state dimension

nu = 2  # input dimension


r = 1.2  # 1.5m
# System states and controls
x = SX.sym("x", nx)
# states of the system

x_ob = SX.sym('x_ob', nu);    # states of obstacle of the system

x_ob_a = SX.sym('x_ob', nu);    # states of artificial obstacle of the system

s = SX.sym("s", 1)  # reference traj variable

u = SX.sym("u", nu)
# control of the system

w = SX.sym("w", 1)  # variable for s_dot = w

dx = mobile_robot_ode(x, u)

ref = reference_traj(s)

# Create the CasADi function
eta = Function("eta", [s], [ref])
system = Function("sys", [x, u], [dx])

# Create CasADi function for obstacle cost

# Declear empty sys matrices
U = SX.sym("U", nu, N)  # Decision variables (controls)

U_a = SX.sym("U", nu, N)  # Decision variables (artificial controls)

X = SX.sym("X", nx, (N + 1))  # Decision variables (states)

X_a = SX.sym("X", nx, (N + 1))  # Decision variables (artificial states)


S_a = SX.sym("S_a", N + 1, 1)  # Decision variable for ref traj

W = SX.sym("W", 1, N)  # Decision variable

umax = np.array([1, 1])

# Parameters:

P_a = SX.sym("P_a", nx+1)  # initial state paraemeter

obs_list = SX.sym("obs",2,3)

# P_obs = SX.sym('P_obs',nu, (N+1))  # obstacle list

# tuning/weight matrices
Q = np.diag([10, 10, 0])
lb_u = np.array([0, -1])
ub_u = np.array([1, 1])
K = np.diag([0.5, 0.5, 0])

R = np.diag([10, 10])

S = np.diag([0.01, 0.01])

mu = 10

T = 10

# Define the stage cost and terminal cost

V_dyn = bilin(Q, x) + bilin(R, u) + bilin(T, (1 - w))

V_a = bilin(K, x) + bilin(S, u)


dyn_cost_fcn = Function("dy_cost", [x, u, w], [V_dyn])
art_cost_fcn = Function("a_cost", [x, u], [V_a])


# Input constraints
lb_u = np.array([0, -1])
ub_u = np.array([1, 1])
lb_x = np.array([-5.5, -2.5])
ub_x = np.array([6.5, 3.5])
lb_xa = np.array([-5.5, -2.5])
ub_xa = np.array([6.5, 3.5])
lb_w = 0
ub_w = 1
# constraints on S
lb_s = 0
ub_s = 90

Opt_Vars = vertcat(
    reshape(X, -1, 1),
    reshape(X_a, -1, 1),
    reshape(U, -1, 1),
    reshape(U_a, -1, 1),
    reshape(W, -1, 1),
    reshape(S_a, -1, 1),
)


def objective_cost():
    J = 0.0
    for i in range(N):
        dx = X[:, i] - X_a[:, i]
        du = U[:, i] - U_a[:, i]
        dx_a = X_a[:, i] - eta(S_a[i])
        du_a = U_a[:, i] - umax
        J += dyn_cost_fcn(dx, du, W[i]) + art_cost_fcn(dx_a, du_a) + obstacle_cost(X[:2, i], X_a[:2, i],obs_list)
    deta_Na = X_a[:, N] - eta(S_a[N])
    deta_N = X[:, N] - X_a[:, N]
    J += bilin(Q, deta_N) + bilin(K, deta_Na) + obstacle_cost(X[:2, N], X_a[:2, N],obs_list)
    return J


def equality_constraints():
    g = []  # Equality constraints initialization
    g.append(X[:, 0] - P_a[:nx])  # Initial state constraint
    g.append(X_a[:, 0] - P_a[:nx])  # Initial artificial state constraint
    g.append(S_a[0] - P_a[nx:])
    for i in range(N):
        st = X[:, i]
        cons = U[:, i]
        st_next_euler = rk4(system, Ts, st, cons)
        st_next = X[:, i + 1]
        g.append(st_next - st_next_euler)
        g.append(S_a[i + 1] - S_a[i] - Ts * W[i])
    return g


def inequality_constraints():

    hu = []  # Box constraints on input
    hu_a = []  # Box constraints on artificial input
    hs = []  # Box constraints on s
    hw = []
    hx = []
    hx_a = []
    for i in range(N):

        # hx.append(1 - X[:1, i] ** 2 - X[1:2, i] ** 2)
        # hx_a.append(1 - X_a[:1, i] ** 2 - X_a[1:2, i] ** 2)
        # hx.append(lb_x-X[:2, i])
        # hx.append(X[:2,i]-ub_x)
        # hx_a.append(lb_xa - X_a[:2, i])
        # hx_a.append(X_a[:2, i] - ub_xa)
        hu.append(lb_u - U[:, i])
        hu.append(U[:, i] - ub_u)
        hu_a.append(lb_u - U_a[:, i])
        hu_a.append(U_a[:, i] - ub_u)
        hs.append(lb_s - S_a[i])
        hs.append(S_a[i] - ub_s)
        hw.append(lb_w - W[i])
        hw.append(W[i] - ub_w)
    hs.append(S_a[N - 1] - S_a[N] + lmda)
    # hx.append(1 - X[:1, N] ** 2 - X[1:2, N] ** 2)
    # hx_a.append(1 - X_a[:1, N] ** 2 - X_a[1:2, N] ** 2)
    return hx, hx_a, hu, hu_a, hs, hw


def Pi_opt_formulation():
    J = objective_cost()
    g = equality_constraints()
    G = vertcat(*g)
    hx, hx_a, hu, hu_a, hs, hw = inequality_constraints()
    Hx = vertcat(*hx, *hx_a)
    Hs = vertcat(*hs)
    Hu = vertcat(*hu, *hu_a)
    Hw = vertcat(*hw)
    G_vcsd = vertcat(*g, *hx, *hx_a, *hu, *hu_a, *hw, *hs)
    lbg = [0] * G.shape[0] + [-np.inf] * (
        Hx.shape[0] + Hu.shape[0] + Hs.shape[0] + Hw.shape[0]
    )
    ubg = [0] * G.shape[0] + [0] * (
        Hx.shape[0] + Hu.shape[0] + Hs.shape[0] + Hw.shape[0]
    )
    lbg_vcsd = vertcat(*lbg)
    ubg_vcsd = vertcat(*ubg)

    opts_setting = {
        "ipopt.max_iter": 500,
        "ipopt.print_level": 4,
        "print_time": 1,
        "ipopt.acceptable_tol": 1e-6,
        "ipopt.acceptable_obj_change_tol": 1e-6,
    }
    vnlp_prob = {
        "f": J,
        "x": Opt_Vars,
        "p": vertcat(P_a,reshape(obs_list,-1,1)),
        "g": G_vcsd,
    }
    pisolver = nlpsol("vsolver", "ipopt", vnlp_prob)
    return lbg_vcsd, ubg_vcsd, G_vcsd, pisolver


lbg_vcsd, ubg_vcsd, G_vcsd, pisolver = Pi_opt_formulation()


def run_open_loop_mpc(x0,s0,x_st_0, x_st_0_a, u_st_0, u_st_0_a, w_st_0, s_st_0, solver,obs):

    args_p = np.append(x0,s0)
    args_p = vertcat(*args_p, *obs)
    args_x0 = np.concatenate(
        [
            x_st_0.T.reshape(-1),
            x_st_0_a.T.reshape(-1),
            u_st_0.T.reshape(-1),
            u_st_0_a.T.reshape(-1),
            w_st_0.T.reshape(-1),
            s_st_0.T.reshape(-1),
        ]
    )
    # Solve the optimization problem
    sol = solver(x0=args_x0, p=args_p, lbg=lbg_vcsd, ubg=ubg_vcsd)
    usol = np.array(sol["x"][2 * nx * (N + 1) : nu * N + 2 * nx * (N + 1)]).reshape((N, nu))
    # Extract the control inputs from the solution
    # extract predicted state
    x_pred = np.array(sol["x"][: nx * (N + 1)]).reshape((N + 1, nx))
    x_pred_a = np.array(sol["x"][nx * (N + 1) : 2 * nx * (N + 1)]).reshape((N + 1, nx))

    x_pred = np.array(sol["x"][: nx * (N + 1)]).reshape((N + 1, nx))
    x_pred_a = np.array(sol["x"][nx * (N + 1) : 2 * nx * (N + 1)]).reshape((N + 1, nx))
    usol = np.array(sol["x"][2 * nx * (N + 1) : nu * N + 2 * nx * (N + 1)]).reshape((N, nu))
    usol_a = np.array(sol["x"][nu * N + 2 * nx * (N + 1) : nu * N + 2 * nx * (N + 1) + N * nu]).reshape((N, nu))
    w = np.array(sol["x"][nu * N + 2 * nx * (N + 1) + N * nu : nu * N + 2 * nx * (N + 1) + N * nu + N])
    s = np.array(sol["x"][nu * N + 2 * nx * (N + 1) + N * nu + N :])

    return x_pred, x_pred_a, usol, usol_a, w, s


u0 = np.array([0.5, 2])
u0_a = np.array([0.5, 2])
x0 = np.array([4,-1, 0])
x0_a = np.array([4,-1, 0])
w0 = 1
s0 = 0

# x_pred, x_pred_a, usol, usol_a, w, s = run_open_loop_mpc(x0, x0_a, u0 ,u0_a, s0, w0,  pisolver )

x_hist = [x0]
x_hist_a = []
u_hist = []
s_hist = [s0]
w_hist = [w0]

u_st_0 = np.tile(u0, (N, 1))
u_st_0_a = np.tile(u0_a, (N, 1))
x_st_0 = np.tile(x0, (N + 1, 1)).T
x_st_0_a = np.tile(x0_a, (N + 1, 1)).T
w_st_0 = np.tile(w0, (N, 1))
s_st_0 = np.tile(s0, (N + 1, 1))

N_sim = 180*(5)

plt.figure(figsize=(10, 6))

data_log = pd.read_csv('nmpc_data_log_2.csv')

s_data = data_log['s'].values
x_data = data_log['x'].values
y_data = data_log['y'].values
theta_data = data_log['theta'].values
obs1_x = data_log['obs1_x'].values
obs1_y = data_log['obs1_y'].values
obs1_r = data_log['obs1_r'].values
obs2_x = data_log['obs2_x'].values
obs2_y = data_log['obs2_y'].values
obs2_r = data_log['obs2_r'].values

def find_obs(s):
    ind = np.argmin(np.abs(s_data-s))

    obs = np.zeros((2, 3))

    obs[0] = [obs1_x[ind],obs1_y[ind],obs1_r[ind]]

    obs[1] = [obs2_x[ind],obs2_y[ind],obs2_r[ind]]

    return obs

for i in range(N_sim):

    obs = find_obs(s0)

    x_pred, x_pred_a, usol, usol_a, w, s = run_open_loop_mpc(x0,s0, x_st_0, x_st_0_a, u_st_0, u_st_0_a, w_st_0, s_st_0, pisolver,np.transpose(obs).reshape((1, -1)))    

    # plt.plot(x_pred[:,0], x_pred[:,1], color='r')

    _, x0, _ = shift(0.5/5, 0, x0, usol, system)
    w0 = w[0]
    s0 =  s0 + (0.5/5)*w0

    x_hist = np.vstack((x_hist,x0.T))
    s_hist = np.vstack((s_hist, s[0]))
    w_hist = np.vstack((w_hist, w[0]))

    if i==0:
        u_hist = usol[0, :].T
    else:
        u_hist = np.vstack((u_hist,usol[0, :].T))

    u_st_0 = np.vstack((usol[1:], usol[-1]))
    u_st_0_a = np.vstack((usol_a[1:], usol_a[-1]))
    x_st_0 = np.vstack((x_pred[1:], x_pred[-1]))
    x_st_0_a = np.vstack((x_pred_a[1:], x_pred_a[-1]))
    w_st_0 = np.vstack((w[1:], w[-1]))
    s_st_0 = np.vstack((s[1:], s[-1]))

eta_val  = []
for  eta in range(90): 
    val = reference_traj(eta)
    eta_val.append(val)
eta_val = np.array(eta_val).reshape(90,3)

# plt.figure(figsize=(10, 6))
# plt.plot(eta_val[:,0], eta_val[:,1], label='Reference Trajectory', color='b')
# plt.plot(x_pred[:,0], x_pred[:,1], label='Open loop Trajectory', color='r')
# plt.plot(x_pred_a[:,0], x_pred_a[:,1], label='Open loop artificial Trajectory', color='k')
# circle = Circle((0, 0), 1, color='g', fill=False, linestyle='--', linewidth=2, label='Obstacle Boundary')
# plt.gca().add_patch(circle)
# plt.xlabel('eta1')
# plt.ylabel('eta2')
# plt.title('Plot of eta1 vs eta2')
# plt.grid(True)
# plt.legend()

# fig, axs = plt.subplots(7, 1, figsize=(15, 15))
# axs[0].plot(x_pred[:, 0])
# axs[0].set_title("r_x")

# axs[1].plot(x_pred[:, 1])
# axs[1].set_title("r_y")

# axs[2].plot(x_pred[:, 2])
# axs[2].set_title("theta")

# axs[3].plot(usol[:, 0])
# axs[3].set_title("u_1")

# axs[4].plot(usol[:, 1])
# axs[4].set_title("u_2")

# axs[5].plot(w)
# axs[5].set_title("w")

# axs[6].plot(s)
# axs[6].set_title("s")

# Load the data from the CSV file


# Assuming the CSV columns are named as 's', 'x', 'y', 'theta', adjust as necessary


plt.plot(eta_val[:,0], eta_val[:,1], label='Reference Trajectory', color='b')
plt.plot(x_hist[:,0], x_hist[:,1], label='Closed loop Trajectory', color='r')
plt.plot(x_data,y_data,label='ROS Closed loop Trajectory', color='c')
circle1 = Circle((0, 0), 1, color='g', fill=False, linestyle='--', linewidth=2, label='Obstacle Boundary')
plt.gca().add_patch(circle1)
circle2 = Circle((4, 3.5), 1, color='g', fill=False, linestyle='--', linewidth=2)
plt.gca().add_patch(circle2)
rectangle = Rectangle((-5.5, -2.5), 12, 6, color='g', fill=False, linestyle='--', linewidth=2)
# plt.gca().add_patch(rectangle)
plt.xlabel('x')
plt.ylabel('y')
plt.title('Trajectories')
plt.grid(True)
plt.legend()

# Plotting the x_hist and u_hist values along with the reference trajectory
fig, axs = plt.subplots(2, 1, figsize=(15, 15))

# Plot x_hist[:, 0] and reference x
axs[0].plot(s_hist,x_hist[:, 0], label='Closed Loop Trajectory x', color='r')
axs[0].plot(eta_val[:len(x_hist), 0], label='Reference x', color='b', linestyle='--')
axs[0].plot(s_data, x_data, label='ROS Closed Loop Trajectory x', color='g', linestyle='-.')  # Plotting CSV data
axs[0].set_title("x vs s", fontsize=15)
axs[0].legend(fontsize="16")
axs[0].tick_params(axis='both', which='major', labelsize=22)

# Plot x_hist[:, 1] and reference y
axs[1].plot(s_hist,x_hist[:, 1], label='Closed Loop Trajectory y', color='r')
axs[1].plot(eta_val[:len(x_hist), 1], label='Reference y', color='b', linestyle='--')
axs[1].plot(s_data, y_data, label='ROS Closed Loop Trajectory y', color='g', linestyle='-.')  # Plotting CSV data
axs[1].set_title("y vs s", fontsize=15)
axs[1].legend(fontsize="16")
axs[1].tick_params(axis='both', which='major', labelsize=22)

# # Plot x_hist[:, 2]
# axs[2].plot(s_hist,x_hist[:, 2], label='theta', color='r')
# axs[2].set_title("theta")

# # Plot u_hist[:, 0]
# axs[3].plot(s_hist[:-1],u_hist[:, 0], label='u_1', color='r')
# axs[3].set_title("u_1")

# # Plot u_hist[:, 1]
# axs[4].plot(s_hist[:-1],u_hist[:, 1], label='u_2', color='r')
# axs[4].set_title("u_2")

# # Plot w_hist
# axs[5].plot(s_hist,w_hist, label='w', color='r')
# axs[5].set_title("w")

# # Plot s_hist
# axs[6].plot(s_hist,s_hist, label='s', color='r')
# axs[6].set_title("s")

plt.tight_layout()
plt.show()
