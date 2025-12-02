# Controllers

This page presents the theoretical foundations of the three trajectory tracking controllers implemented for the Benji rover. All controllers share a common structure: a feedforward term $\mathbf{u}_{ref}$ from the reference trajectory plus a feedback correction $\delta\mathbf{u}$ computed by the controller. The key distinction lies in how each controller computes the feedback term.

## Control Problem Statement

Given:
- Current state estimate $\mathbf{x}_k$
- Reference trajectory $\{\mathbf{x}_{ref,k}, \mathbf{u}_{ref,k}\}_{k=0}^{N}$
- Input constraints $|\mathbf{u}| \leq u_{max}$

Find the control input $\mathbf{u}_k$ that drives the tracking error $\mathbf{e}_k = \mathbf{x}_k - \mathbf{x}_{ref,k}$ to zero while respecting actuator limits.

The control law takes the form:

$$\mathbf{u} = \mathbf{u}_{ref} + \delta\mathbf{u}$$

where $\mathbf{u}_{ref}$ is the feedforward (open-loop) component and $\delta\mathbf{u}$ is the feedback (closed-loop) correction. The feedforward term is essential: it carries the nominal control effort, while feedback handles disturbances and model errors. Controllers that rely purely on feedback will always lag behind the reference.

## LQR with Feedforward

The Linear Quadratic Regulator (LQR) provides an optimal linear state feedback gain for the linearized error dynamics. This is the simplest and fastest of the three controllers.

### Problem Formulation

LQR minimizes the infinite-horizon quadratic cost:

$$J = \sum_{k=0}^{\infty} \left( \mathbf{e}_k^T Q \mathbf{e}_k + \delta\mathbf{u}_k^T R \, \delta\mathbf{u}_k \right)$$

subject to the discrete-time linearized error dynamics:

$$\mathbf{e}_{k+1} = A_d \mathbf{e}_k + B_d \delta\mathbf{u}_k$$

The weight matrices $Q \succeq 0$ and $R \succ 0$ encode the relative importance of tracking accuracy versus control effort. Larger diagonal entries in $Q$ increase sensitivity to the corresponding state error; larger entries in $R$ reduce control authority.

### Optimal Gain Computation

The optimal feedback gain $K$ is obtained by solving the Discrete-time Algebraic Riccati Equation (DARE):

$$P = A_d^T P A_d - A_d^T P B_d (R + B_d^T P B_d)^{-1} B_d^T P A_d + Q$$

The gain matrix is then:

$$K = (R + B_d^T P B_d)^{-1} B_d^T P A_d$$

### Control Law

The LQR+FF control law is:

$$\mathbf{u} = \mathbf{u}_{ref} - K \mathbf{e}$$

where the error $\mathbf{e} = \mathbf{x} - \mathbf{x}_{ref}$ uses angle wrapping on the heading component (see [Dynamics](dynamics.md)).

### Linearization About the Reference Trajectory

The error dynamics formulation linearizes about the reference trajectory, not about a fixed operating point. The $A_d$ matrix depends on the reference state at each timestep:

$$A_d = A_d(\mathbf{x}_{ref,k})$$

This yields a time-varying linear system that accurately captures the dynamics along the reference. Conceptually, we are always linearizing at the point where we expect to be, which is where the linearization is most valid.

For LQR, there are two approaches:

1. **Fixed-gain LQR**: Compute $K$ once using nominal reference values. This is simpler and often sufficient when the reference trajectory has consistent characteristics (e.g., roughly constant velocity, moderate turns).

2. **Time-varying LQR**: Recompute $A_d$ and $K$ at each timestep using the current reference state. This provides optimal gains throughout the trajectory but requires solving the DARE online.

For Benji, the fixed-gain approach is used for LQR. The gain is computed at nominal cruise velocity, which provides acceptable performance for smooth trajectories. Note that MPC naturally handles this better: it relinearizes the dynamics along the prediction horizon at each timestep.

### Limitations

LQR+FF has two key limitations:

1. **No constraint handling**: Input saturation is applied after the control law, which can degrade performance when constraints are active.

2. **No preview**: The controller only sees the current error, not the upcoming trajectory. It cannot anticipate turns or prepare for constraints.

## Linear MPC

Model Predictive Control (MPC) addresses both limitations of LQR by optimizing over a finite horizon with explicit constraints. Linear MPC uses the linearized error dynamics, yielding a convex optimization problem that can be solved efficiently.

### Problem Formulation

At each timestep, Linear MPC solves:

$$\min_{\delta\mathbf{u}_{0:N-1}} \sum_{k=0}^{N} \mathbf{e}_k^T Q \mathbf{e}_k + \sum_{k=0}^{N-1} \delta\mathbf{u}_k^T R \, \delta\mathbf{u}_k + \sum_{k=0}^{N-1} \Delta\mathbf{u}_k^T R_{rate} \Delta\mathbf{u}_k$$

subject to:

$$\mathbf{e}_{k+1} = A_d \mathbf{e}_k + B_d \delta\mathbf{u}_k, \quad k = 0, \ldots, N-1$$

$$|\mathbf{u}_{ref,k} + \delta\mathbf{u}_k| \leq u_{max}, \quad k = 0, \ldots, N-1$$

where $N$ is the prediction horizon.

### Rate Penalty

The rate penalty term $\Delta\mathbf{u}_k^T R_{rate} \Delta\mathbf{u}_k$ penalizes changes in control input between timesteps:

$$\Delta\mathbf{u}_k = \mathbf{u}_k - \mathbf{u}_{k-1}$$

This term is critical for systems with actuator dynamics. The motor time constant $\tau$ limits how quickly the wheel velocities can change. Commanding rapid control variations results in the actual wheel velocities lagging behind the commands, degrading tracking performance. The rate penalty encourages control sequences that the actuators can actually follow.

### Relinearization

Unlike fixed-gain LQR, Linear MPC relinearizes the error dynamics along the prediction horizon at each control update. When the reference trajectory is updated, the linearization is updated accordingly. This yields a time-varying linear system:

$$\mathbf{e}_{k+1} = A_d(\mathbf{x}_{ref,k}) \mathbf{e}_k + B_d \delta\mathbf{u}_k$$

where $A_d(\mathbf{x}_{ref,k})$ is evaluated at the reference state for each step in the horizon. This is the key advantage over fixed-gain LQR: the linearization remains valid along the entire prediction horizon, even when the reference trajectory involves significant heading changes.

### Receding Horizon

MPC uses a receding horizon strategy:

1. Solve the optimization to obtain $\delta\mathbf{u}_{0:N-1}^*$
2. Apply only the first control: $\mathbf{u} = \mathbf{u}_{ref,0} + \delta\mathbf{u}_0^*$
3. Shift the horizon forward and repeat

This provides implicit feedback: the optimization is re-solved at each timestep with updated state and reference information, allowing the controller to react to disturbances and adapt to trajectory changes.

## Nonlinear MPC

Nonlinear MPC uses the full nonlinear dynamics for prediction, avoiding the linearization error inherent in Linear MPC. This comes at the cost of solving a non-convex optimization problem.

### Problem Formulation

NL-MPC solves:

$$\min_{\mathbf{u}_{0:N_c-1}} \sum_{k=0}^{N_p} \|\mathbf{x}_k - \mathbf{x}_{ref,k}\|_Q^2 + \sum_{k=0}^{N_c-1} \|\mathbf{u}_k - \mathbf{u}_{ref,k}\|_R^2$$

subject to:

$$\mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k), \quad k = 0, \ldots, N_p-1$$

$$|\mathbf{u}_k| \leq u_{max}, \quad k = 0, \ldots, N_c-1$$

where $N_p$ is the prediction horizon and $N_c \leq N_p$ is the control horizon.

### Control Horizon

The control horizon $N_c$ is the number of timesteps over which the control input is optimized. Beyond $N_c$, the control is held constant at $\mathbf{u}_{N_c-1}$. This reduces the number of decision variables from $2 N_p$ to $2 N_c$, significantly decreasing solve time.

The trade-off is reduced flexibility in the control sequence. For trajectory tracking where the reference provides a good nominal input, a short control horizon is often sufficient.

### Solution Methods

The nonlinear optimization can be solved using:

1. **Sequential Quadratic Programming (SQP)**: Iteratively solves QP subproblems that approximate the nonlinear problem. Each iteration requires linearizing the dynamics and solving a QP.

2. **Interior Point Methods**: Applies Newton's method to the KKT conditions with a barrier function for inequality constraints.

3. **Direct Collocation**: Discretizes the dynamics using collocation points and solves the resulting large sparse NLP.

For Benji, a direct single-shooting approach with SLSQP (Sequential Least Squares Quadratic Programming) is used. The states are computed by forward simulation from the control sequence, reducing the decision variables to only the controls.

### Local Minima

Unlike convex Linear MPC, nonlinear MPC may converge to local minima. The optimization landscape depends on:

- Initial guess quality (warm starting helps)
- Horizon length (longer horizons have more complex landscapes)
- Constraint geometry

For trajectory tracking with good feedforward, local minima are rarely problematic because the initial guess (shifted previous solution or feedforward sequence) is typically close to the global optimum.

### Computational Considerations

NL-MPC is approximately 10x slower than Linear MPC for this system. The dominant cost is evaluating the dynamics and their gradients at each iteration. Strategies to reduce computation include:

- Shorter control horizon $N_c$
- Approximate gradients (finite differences vs. analytical)
- Early termination with suboptimality tolerance
- Reduced prediction horizon $N_p$

## Comparison and Selection Guidelines

| Aspect | LQR+FF | Linear MPC | NL-MPC |
|--------|--------|------------|--------|
| Dynamics model | Linearized (fixed) | Linearized (time-varying) | Full nonlinear |
| Constraint handling | Post-hoc saturation | Explicit in optimization | Explicit in optimization |
| Preview capability | None | Full horizon | Full horizon |
| Problem type | Algebraic (offline) | Convex optimization | Non-convex NLP |
| Solve time | < 0.1 ms | ~2 ms | ~20 ms |
| Embedded suitability | Excellent | Good | Challenging |
| Optimality | Global (for linear system) | Global (for linearized system) | Local |

### Selection Criteria

**Use LQR+FF when:**
- Computational resources are extremely limited
- Trajectories are smooth with gradual turns
- Constraints are rarely active
- Simplicity is valued over optimality

**Use Linear MPC when:**
- Input constraints must be respected proactively
- Preview of upcoming trajectory is beneficial
- Real-time embedded deployment is required
- Linearization error is acceptable

**Use NL-MPC when:**
- Trajectories involve large heading changes
- Maximum tracking accuracy is required
- Computational budget allows 10-20 ms solve times
- Linearization error significantly degrades Linear MPC performance

### Tuning Recommendations

For all controllers:

1. **Start with feedforward**: Ensure the reference trajectory is physically realizable and $\mathbf{u}_{ref}$ is correctly computed. Poor feedforward cannot be compensated by feedback.

2. **Weight matrix selection**: Begin with identity matrices scaled by the inverse of expected error magnitudes squared. For example, if position errors should be within 10 cm, set $Q_{xx} = Q_{yy} \approx 1/(0.1)^2 = 100$.

3. **Rate penalty (MPC)**: Set $R_{rate}$ based on actuator bandwidth. For motor time constant $\tau$, commands changing faster than $1/\tau$ cannot be tracked. A reasonable starting point is $R_{rate} \approx R \cdot \tau / \Delta t$.

4. **Horizon length (MPC)**: The horizon should capture the relevant dynamics. For trajectory tracking, 0.5-1.0 seconds is typically sufficient. Longer horizons increase computation without proportional benefit.

## References

The theoretical foundations of these controllers are covered in standard texts:

- LQR and optimal control: Anderson & Moore, *Optimal Control: Linear Quadratic Methods*
- MPC fundamentals: Rawlings, Mayne & Diehl, *Model Predictive Control: Theory, Computation, and Design*
