class MobileRobotMPC:
    def __init__(self, Ts=0.3, N=20, nx=3, nu=2):
        # Controller parameters
        self.Ts_sim = 0.05
        self.Ts = Ts
        self.N = N
        self.nx = nx
        self.nu = nu

        self.reference_function = f_s

        self.lmda = 0.05

        self.lb_u = np.array([0, -1])  # Lower bounds
        self.ub_u = np.array([1, 1])   # Upper bounds
        self.lb_w = 0
        self.ub_w = 1
        # constraints on S
        self.lb_s = 0
        self.ub_s = path_segments[-1].end_time

        self.umax = np.array([1, 1])

        # Initial states and controls
        self.initialize_variables()
        self.setup_weights()

        # Create NLP solver
        self.lbg, self.ubg, self.g_constraints, self.solver = self.setup_nlp_solver()

    def initialize_variables(self):
        initial_point = f_s(0).full().flatten()
        self.x0 = np.array([initial_point[0], initial_point[1], 0])
        self.x0_a = self.x0.copy()
        self.u0 = np.array([0, 0])
        self.u0_a = self.u0.copy()
        self.w0 = 1
        self.s0 = 0
        self.x_st_0 = np.tile(self.x0, (self.N + 1, 1)).T
        self.x_st_0_a = np.tile(self.x0_a, (self.N + 1, 1)).T
        self.u_st_0 = np.tile(self.u0, (self.N, 1))
        self.u_st_0_a = np.tile(self.u0_a, (self.N, 1))
        self.w_st_0 = np.tile(self.w0, (self.N, 1))
        self.s_st_0 = np.tile(self.s0, (self.N + 1, 1))

    def setup_weights(self):
        self.Q = np.diag([100, 100, 0])  # Adjust the weights as needed for state deviation
        self.K = np.diag([2.5, 2.5, 0])
        self.R = np.diag([0.1, 0.1])  # Control effort weights
        self.S = np.diag([5, 5])
        self.T = 10
        self.mu = 5*10**6

    def objective_cost(self, X, X_a, U, U_a, W, S_a):
        J = 0.0
        for i in range(self.N):
            dx = X[:, i] - X_a[:, i]
            du = U[:, i] - U_a[:, i]
            dx_a = X_a[:, i] - self.reference_function(S_a[i])
            du_a = U_a[:, i] - self.umax

            J += ca.mtimes(dx.T, self.Q @ dx) + ca.mtimes(du.T, self.R @ du) + (1-W[i])**2*self.T
            J += ca.mtimes(dx_a.T, self.K @ dx_a) + ca.mtimes(du_a.T, self.S @ du_a)
            J += self.obstacle_cost(X[:2, i], X_a[:2, i])

        dx_N = X[:, self.N] - X_a[:, self.N]
        dx_a_N = X_a[:, self.N] - self.reference_function(S_a[self.N])

        J += ca.mtimes(dx_N.T, self.Q @ dx_N) + ca.mtimes(dx_a_N.T, self.K @ dx_a_N)
        J += self.obstacle_cost(X[:2, self.N], X_a[:2, self.N])

        return J

    def equality_constraints(self, X, X_a, U, S_a, P_a, W):
        g = [X[:, 0] - P_a[:self.nx], X_a[:, 0] - P_a[:self.nx], S_a[0] - P_a[self.nx]]
        for i in range(self.N):
            st = X[:, i]
            cons = U[:, i]
            st_next_euler = self.rk4(st, cons)
            st_next = X[:, i + 1]
            g.append(st_next - st_next_euler)
            g.append(S_a[i + 1] - S_a[i] - self.Ts * W[i])
        return g

    def inequality_constraints(self, U, U_a, W, S_a):
        hu, hu_a, hw, hs = [], [], [], []
        for i in range(self.N):
            hu.append(self.lb_u - U[:, i])
            hu.append(U[:, i] - self.ub_u)
            hu_a.append(self.lb_u - U_a[:, i])
            hu_a.append(U_a[:, i] - self.ub_u)
            hs.append(self.lb_s - S_a[i])
            hs.append(S_a[i] - self.ub_s)
            hw.append(self.lb_w - W[i])
            hw.append(W[i] - self.ub_w)
        hs.append(S_a[self.N - 1] - S_a[self.N] + self.lmda)

        return hu,hu_a, hw, hs

    def setup_nlp_solver(self):
        # Define decision variables
        X = ca.SX.sym("X", self.nx, self.N + 1)
        X_a = ca.SX.sym("X_a", self.nx, self.N + 1)
        U = ca.SX.sym("U", self.nu, self.N)
        U_a = ca.SX.sym("U_a", self.nu, self.N)
        W = ca.SX.sym("W", self.N)
        S_a = ca.SX.sym("S_a", self.N + 1, 1)
        P_a = ca.SX.sym("P_a", self.nx + 1)

        # Objective function
        J = self.objective_cost(X, X_a, U, U_a, W, S_a)

        # Constraints
        g_eq = self.equality_constraints(X, X_a, U, S_a, P_a, W)
        G = ca.vertcat(*g_eq)

        hu, hu_a, hw, hs = self.inequality_constraints(U, U_a, W, S_a)

        Hs = ca.vertcat(*hs)
        Hu = ca.vertcat(*hu, *hu_a)
        Hw = ca.vertcat(*hw)

        G_vcsd = ca.vertcat(*g_eq, *hu, *hu_a, *hw, *hs)

        lbg = [0] * G.shape[0] + [-np.inf] * (
            Hu.shape[0] + Hs.shape[0] + Hw.shape[0]
        )
        ubg = [0] * G.shape[0] + [0] * (
            Hu.shape[0] + Hs.shape[0] + Hw.shape[0]
        )

        lbg_vcsd = ca.vertcat(*lbg)
        ubg_vcsd = ca.vertcat(*ubg)

        # Define optimization variables
        Opt_Vars = ca.vertcat(
            ca.reshape(X, -1, 1),
            ca.reshape(X_a, -1, 1),
            ca.reshape(U, -1, 1),
            ca.reshape(U_a, -1, 1),
            ca.reshape(W, -1, 1),
            ca.reshape(S_a, -1, 1)
        )

        opts_setting = {
            "ipopt.max_iter": 500,
            "ipopt.print_level": 4,
            "print_time": 1,
            "ipopt.acceptable_tol": 1e-6,
            "ipopt.acceptable_obj_change_tol": 1e-6,
        }
        vnlp_prob = {"f": J, "x": Opt_Vars, "p": P_a, "g": G_vcsd}
        solver = ca.nlpsol("solver", "ipopt", vnlp_prob, opts_setting)
        return lbg_vcsd, ubg_vcsd, G_vcsd, solver

    def rk4(self, x, u):
        h = self.Ts
        k1 = self.mobile_robot_ode(x, u)
        k2 = self.mobile_robot_ode(x + h / 2 * k1, u)
        k3 = self.mobile_robot_ode(x + h / 2 * k2, u)
        k4 = self.mobile_robot_ode(x + h * k3, u)
        return x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    def mobile_robot_ode(self, x, u):
        dx1 = u[0] * ca.cos(x[2])
        dx2 = u[0] * ca.sin(x[2])
        dx3 = u[1]
        return ca.vertcat(dx1, dx2, dx3)

    def obstacle_cost(self, x_ob, x_ob_a):
        r = 0.15
        h_ob = ca.fmax(r**2 - (x_ob[0]-3)**2 - (x_ob[1]-0.35)**2, 0)
        h_ob_a = ca.fmax(r**2 - (x_ob_a[0]-3)**2 - (x_ob_a[1]-0.35)**2, 0)
        return 0.5 * self.mu * h_ob**2 + 0.5 * self.mu * h_ob_a**2

    
    def run_open_loop_mpc(self, x0, s0):
        """
        Runs the MPC for a single open-loop iteration given initial states.
        
        Args:
            x0 (np.array): Initial state of the robot.
            s0 (float): Initial value of the trajectory parameter.
        
        Returns:
            tuple: Predicted state trajectory (x_pred), predicted artificial state trajectory (x_pred_a),
                control inputs (usol), artificial control inputs (usol_a), w values (wsol), and s values (s).
        """
        # Prepare the parameter vector with initial state and initial s
        args_p = np.append(x0, s0)
        args_p = ca.vertcat(*args_p)

        # Prepare the initial guess for the decision variables
        args_x0 = np.concatenate(
            [self.x_st_0.T.reshape(-1), self.x_st_0_a.T.reshape(-1),
            self.u_st_0.T.reshape(-1), self.u_st_0_a.T.reshape(-1),
            self.w_st_0.T.reshape(-1), self.s_st_0.T.reshape(-1)]
        )

        # Solve the optimization problem
        sol = self.solver(x0=args_x0, p=args_p, lbg=self.lbg, ubg=self.ubg)

        # Extract predicted state trajectories and control inputs
        x_pred = np.array(sol["x"][: self.nx * (self.N + 1)]).reshape((self.N + 1, self.nx))
        x_pred_a = np.array(sol["x"][self.nx * (self.N + 1): 2 * self.nx * (self.N + 1)]).reshape((self.N + 1, self.nx))
        usol = np.array(sol["x"][2 * self.nx * (self.N + 1): 2 * self.nx * (self.N + 1) + self.nu * self.N]).reshape((self.N, self.nu))
        usol_a = np.array(sol["x"][2 * self.nx * (self.N + 1) + self.nu * self.N: 2 * self.nx * (self.N + 1) + 2 * self.nu * self.N]).reshape((self.N, self.nu))
        wsol = np.array(sol["x"][2 * self.nx * (self.N + 1) + 2 * self.nu * self.N: 2 * self.nx * (self.N + 1) + 2 * self.nu * self.N + self.N])
        ssol = np.array(sol["x"][-(self.N + 1):])

        return x_pred, x_pred_a, usol, usol_a, wsol, ssol

    
    def plot_results(self, x_pred, x_pred_a, reference_trajectory):
        plt.figure(figsize=(10, 10))

        plt.plot(reference_trajectory[:, 0], reference_trajectory[:, 1], label='Reference Trajectory', color='b')
        plt.plot(x_pred[:, 0], x_pred[:, 1], label='Open-loop Trajectory', color='r')
        plt.plot(x_pred_a[:, 0], x_pred_a[:, 1], label='Artificial Open-loop Trajectory', color='k')

        circle = plt.Circle((3,0.35), 0.1, color='red', alpha=0.5)
        plt.gca().add_patch(circle)

        plt.title('Reference Path, Obstacles, and Optimal Trajectory')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

    def run_mpc_loop(self, max_steps=1000):
        """
        Runs the MPC in a loop for a specified number of steps.
        
        Args:
            max_steps (int): Maximum number of iterations for the MPC loop.
        
        Returns:
            tuple: Histories of state trajectories (x_hist, x_hist_a),
                control inputs (u_hist), trajectory parameter (s_hist),
                and auxiliary variable (w_hist).
        """
        x_hist = [self.x0]
        x_hist_a = [self.x0_a]
        s_hist = [self.s0]
        u_hist = []
        w_hist = [self.w0]

        x_current = self.x0.copy()
        s_current = self.s0

        for step in range(max_steps):
            # Run one iteration of the open-loop MPC
            x_pred, x_pred_a, usol, usol_a, wsol, ssol = self.run_open_loop_mpc(x_current, s_current)

            # Apply the first control input and simulate the system
            u0 = usol[0]
            x_next = self.simulate_kinematic_step(x_current, u0, self.Ts_sim)
            w_next = wsol[0]
            s_next = s_current + self.Ts_sim * w_next

            # Store the results
            x_hist.append(x_next)
            x_hist_a.append(x_pred_a[1])
            s_hist.append(s_next[0])  # Append as a scalar
            u_hist.append(u0)
            w_hist.append(w_next[0])  # Append as a scalar

            # Update the current state and shift the initial guess
            x_current = x_next
            s_current = s_next
            self.u_st_0 = np.vstack((usol[1:], usol[-1]))
            self.u_st_0_a = np.vstack((usol_a[1:], usol_a[-1]))
            self.x_st_0 = np.vstack((x_pred[1:], x_pred[-1]))
            self.x_st_0_a = np.vstack((x_pred_a[1:], x_pred_a[-1]))
            self.w_st_0 = np.vstack((wsol[1:], wsol[-1]))
            self.s_st_0 = np.vstack((ssol[1:], ssol[-1]))

            # Stopping condition: check if the state is close to the target
            if np.linalg.norm(x_next[:2] - self.reference_function(self.ub_s).full().flatten()[:2]) < 0.1:
                print("Reached near the target.")
                break

        # Convert all history lists to numpy arrays
        x_hist = np.array(x_hist)
        x_hist_a = np.array(x_hist_a)
        u_hist = np.array(u_hist)
        s_hist = np.array(s_hist)  # Now should be a uniform 1D array
        w_hist = np.array(w_hist)  # Now should be a uniform 1D array

        return x_hist, x_hist_a, u_hist, s_hist, w_hist

    def simulate_kinematic_step(self, state, control, Ts):
        """
        Simulates the next state using a kinematic model and control input.
        """
        x, y, theta = state
        v, omega = control
        x_next = x + Ts * v * np.cos(theta)
        y_next = y + Ts * v * np.sin(theta)
        theta_next = theta + Ts * omega
        return np.array([x_next, y_next, theta_next])

# Run the loop
if __name__ == "__main__":
    mpc = MobileRobotMPC()
    reference_points = [f_s(i).full().flatten() for i in range(60)]
    reference_trajectory = np.array(reference_points)
    x_hist, x_hist_a, u_hist, s_hist, w_hist = mpc.run_mpc_loop(max_steps=1200)

    # Plot results
    mpc.plot_results(np.array(x_hist), np.array(x_hist_a), reference_trajectory)