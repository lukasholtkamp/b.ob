class NMPCController:
    def __init__(self):
        # NMPC Parameters
        self.Ts = 0.3  # Sampling time
        self.Ts_sim = 0.05  # Smaller simulation step time
        self.N = 20  # Prediction horizon
        self.nx = 4  # State dimension (x, y, theta, s)
        self.nu = 3  # Input dimension (v, omega, w)

        # Initialize `s0` and get the initial state from the first point of `f_s`
        self.s0 = 0
        self.initial_point = f_s(0).full().flatten()  # Store the initial point for reuse
        self.current_state = np.array([self.initial_point[0], self.initial_point[1], 0, self.s0])  # Assume theta = 0

        # Initialize control variables
        self.u0 = np.array([1, 0])
        self.w0 = 1

        # Set obstacle at (3, 0.5) with a radius of 0.03
        self.max_obs = 1
        self.obs_list = np.array([[3.0, 0.3, 0.1]])  # Single obstacle

        # Setup MPC
        self.ocp = self.setup_ocp_with_cost_function()
        self.solver = AcadosOcpSolver(self.ocp, json_file="acados_ocp.json")

        self.closed_loop_trajectory = []

        self.final_position = f_s(path_segments[-1].end_time).full().flatten()


    def mobile_robot_ode(self):
        x = ca.SX.sym("x")
        y = ca.SX.sym("y")
        theta = ca.SX.sym("theta")
        s = ca.SX.sym("s")
        v = ca.SX.sym("v")
        omega = ca.SX.sym("omega")
        w = ca.SX.sym("w")

        states = ca.vertcat(x, y, theta, s)
        controls = ca.vertcat(v, omega, w)
        obs_list = ca.SX.sym("obs", self.max_obs, 3)

        dx = v * ca.cos(theta)
        dy = v * ca.sin(theta)
        dtheta = omega
        ds = w

        xdot = ca.vertcat(dx, dy, dtheta, ds)

        model = AcadosModel()
        model.f_expl_expr = xdot
        model.x = states
        model.u = controls
        model.p = ca.reshape(obs_list, -1, 1)
        model.name = "mobile_robot"

        return model

    def setup_ocp_with_cost_function(self):
        ocp = AcadosOcp()
        model = self.mobile_robot_ode()
        ocp.model = model

        N = self.N
        Ts = self.Ts
        T = N * Ts
        mu = 4 * 10**4

        ocp.dims.N = N
        ocp.solver_options.tf = T

        Q = np.diag([1000, 1000, 0])  # State weights (3x3)
        R = np.diag([0.01, 0.01])     # Control input weights (2x2)
        T_cost = np.array([[10]])     # Weight for time dilation cost (1x1)

        x = ocp.model.x[:3]  # State vector [x, y, theta]
        u = ocp.model.u[:2]  # Control vector [v, omega]
        w = ocp.model.u[2]   # Third control input (scalar)
        s = ocp.model.x[3]   # Path parameter

        obs_list = ca.reshape(ocp.model.p, self.max_obs, 3)

        # Reference trajectory using f_s
        xi_s = f_s(s)
        dx = x - xi_s

        # Cost function expressions
        ocp.model.cost_y_expr = ca.vertcat(dx, u, (1 - w))
        for i in range(self.max_obs):
            h = ca.if_else(
                0.1 > 0,
                ca.fmax((0.1+.08)**2 - (x[0] - 3.0)**2 - (x[1] - 0.3)**2, 0),
                0
            )
            ocp.model.cost_y_expr = ca.vertcat(ocp.model.cost_y_expr, h)

        ocp.model.cost_y_expr_e = ca.vertcat(dx)

        # Create the weight matrix with correct dimensions
        num_obstacle_terms = self.max_obs
        obstacle_weights = 0.5 * mu * np.eye(num_obstacle_terms)

        ocp.cost.W = np.block([
            [Q, np.zeros((3, 2)), np.zeros((3, 1)), np.zeros((3, num_obstacle_terms))],
            [np.zeros((2, 3)), R, np.zeros((2, 1)), np.zeros((2, num_obstacle_terms))],
            [np.zeros((1, 3)), np.zeros((1, 2)), T_cost, np.zeros((1, num_obstacle_terms))],
            [np.zeros((num_obstacle_terms, 3)), np.zeros((num_obstacle_terms, 2)), np.zeros((num_obstacle_terms, 1)), obstacle_weights]
        ])

        ocp.cost.W_e = Q

        ny = ocp.model.cost_y_expr.size()[0]
        ocp.cost.yref = np.zeros((ny,))
        ocp.cost.yref_e = np.zeros((dx.size()[0],))

        ocp.constraints.lbx = np.array([0])  # Lower bound for `s`
        ocp.constraints.ubx = np.array([path_segments[-1].end_time])  # Upper bound for `s`
        ocp.constraints.idxbx = np.array([3])  # Constraining the `s` state only

        ocp.parameter_values = np.zeros((self.max_obs * 3, 1))

        ocp.constraints.lbu = np.array([0, -0.7, 0])
        ocp.constraints.ubu = np.array([1, 0.7, 1])
        ocp.constraints.idxbu = np.array([0, 1, 2])

        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"

        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.tf = T
        ocp.solver_options.nlp_solver_max_iter = 500
        ocp.solver_options.qp_solver_iter_max = 500
        ocp.solver_options.print_level = 0
        ocp.solver_options.regularize_method = "MIRROR"
        ocp.solver_options.levenberg_marquardt = 1e-4

        # Use the stored initial_point for the initial state
        x0_initial = np.array([self.initial_point[0], self.initial_point[1], 0, self.s0])  # Initial state with s0
        ocp.constraints.x0 = x0_initial

        return ocp


    def run_mpc(self):
        self.solver.set(0, "p", self.obs_list.flatten())
        self.solver.set(0, "lbx", self.current_state)
        self.solver.set(0, "ubx", self.current_state)

        status = self.solver.solve()

        if status != 0:
            print(f"ACADOS returned status {status}")

        usol = self.solver.get(0, "u")
        x_opt = [self.solver.get(i, "x") for i in range(self.N + 1)]
        return usol, x_opt


    def plot_results(self, x_opt):
        # Plot the reference path
        s_values = np.linspace(0, path_segments[-1].end_time, 500)
        ref_path = np.array([f_s(s).full().flatten() for s in s_values])
        
        plt.figure(figsize=(10, 10))
        
        # Plot reference path
        plt.plot(ref_path[:, 0], ref_path[:, 1], 'g--', label='Reference Path')

        # Plot obstacles
        for obs in self.obs_list:
            circle = plt.Circle((obs[0], obs[1]), obs[2], color='red', alpha=0.5)
            plt.gca().add_patch(circle)

        # Plot x_opt trajectory
        x_opt = np.array(x_opt)
        plt.plot(x_opt[:, 0], x_opt[:, 1], 'b-', label='Optimal Trajectory')

        # Plot settings
        plt.scatter(self.initial_point[0], self.initial_point[1], color='black', label='Start Point')
        plt.title('Reference Path, Obstacles, and Optimal Trajectory')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

    def simulate_kinematic_step(self, state, control, Ts):
        x, y, theta, s = state
        v, omega = control[:2]
        x_next = x + Ts * v * np.cos(theta)
        y_next = y + Ts * v * np.sin(theta)
        theta_next = theta + Ts * omega
        s_next = s + Ts * control[2]  # Increment s using the third control input (w)
        return np.array([x_next, y_next, theta_next, s_next])
    
    def run_mpc_loop(self):
        # Run the loop until the current position is within 0.1 units of the final position
        while np.linalg.norm(self.current_state[:2] - self.final_position[:2]) > 0.1:
            self.solver.set(0, "p", self.obs_list.flatten())
            self.solver.set(0, "lbx", self.current_state)
            self.solver.set(0, "ubx", self.current_state)

            status = self.solver.solve()

            if status != 0:
                print(f"ACADOS returned status {status}")
                break

            usol = self.solver.get(0, "u")
            x_opt = [self.solver.get(i, "x") for i in range(self.N + 1)]

            # Apply the first control input to the kinematic model
            self.current_state = self.simulate_kinematic_step(self.current_state, usol, self.Ts_sim)

            # Update s0 based on the third control input from usol
            self.s0 = self.current_state[3]

            # Store the current state for plotting
            self.closed_loop_trajectory.append(self.current_state[:3])  # Store (x, y, theta)

    def run_mpc_loop_n(self, n=50):
        for _ in range(n):
            self.solver.set(0, "p", self.obs_list.flatten())
            self.solver.set(0, "lbx", self.current_state)
            self.solver.set(0, "ubx", self.current_state)

            status = self.solver.solve()

            if status != 0:
                print(f"ACADOS returned status {status}")
                break

            usol = self.solver.get(0, "u")
            x_opt = [self.solver.get(i, "x") for i in range(self.N + 1)]

            # Apply the first control input to the kinematic model
            self.current_state = self.simulate_kinematic_step(self.current_state, usol, self.Ts_sim)

            # Update s0 based on the third control input from usol
            self.s0 = self.current_state[3]

            # Store the current state for plotting
            self.closed_loop_trajectory.append(self.current_state[:3])  # Store (x, y, theta)

    def plot_closed_loop(self):
        self.closed_loop_trajectory = np.array(self.closed_loop_trajectory)
        s_values = np.linspace(0, path_segments[-1].end_time, 500)
        ref_path = np.array([f_s(s).full().flatten() for s in s_values])

        plt.figure(figsize=(10, 10))
        plt.plot(ref_path[:, 0], ref_path[:, 1], 'g--', label='Reference Path')

        for obs in self.obs_list:
            circle = plt.Circle((obs[0], obs[1]), obs[2], color='red', alpha=0.5)
            plt.gca().add_patch(circle)

        plt.plot(self.closed_loop_trajectory[:, 0], self.closed_loop_trajectory[:, 1], 'b-', label='Closed-Loop Trajectory')
        plt.scatter(self.initial_point[0], self.initial_point[1], color='black', label='Start Point')
        plt.title('Closed-Loop Trajectory, Reference Path, and Obstacles')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

if __name__ == "__main__":
    controller = NMPCController()
    # usol, x_opt = controller.run_mpc()
    # print("Optimal control input:", usol)
    # print("Optimal state trajectory:", x_opt)

    # # Plot results
    # controller.plot_results(x_opt)

    controller.run_mpc_loop_n(n=1000)
    controller.plot_closed_loop()