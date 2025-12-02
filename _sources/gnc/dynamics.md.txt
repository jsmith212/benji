# Dynamics

This page derives the equations of motion for the tank drive robot and formulates the error dynamics used for trajectory tracking control.

## Nonlinear Equations of Motion

The robot state is $\mathbf{x} = [x, y, \theta, v_l, v_r]^T$ where $(x, y)$ is position, $\theta$ is heading, and $(v_l, v_r)$ are left/right wheel velocities. The control input is commanded wheel velocities $\mathbf{u} = [u_l, u_r]^T$.

### Kinematics

The position and heading evolve according to standard differential drive kinematics:

$$\dot{x} = \frac{v_l + v_r}{2} \cos\theta$$

$$\dot{y} = \frac{v_l + v_r}{2} \sin\theta$$

$$\dot{\theta} = \frac{v_r - v_l}{L}$$

where $L$ is the wheelbase (distance between wheel centers).

### Motor Dynamics

The wheel velocities follow first-order dynamics with time constant $\tau$:

$$\dot{v}_l = \frac{u_l - v_l}{\tau}$$

$$\dot{v}_r = \frac{u_r - v_r}{\tau}$$

This captures the lag between commanded and actual wheel velocities due to motor and drivetrain inertia.

### Compact Form

In vector notation:

$$\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}) = \begin{bmatrix} \frac{v_l + v_r}{2} \cos\theta \\ \frac{v_l + v_r}{2} \sin\theta \\ \frac{v_r - v_l}{L} \\ \frac{u_l - v_l}{\tau} \\ \frac{u_r - v_r}{\tau} \end{bmatrix}$$

## Error Dynamics Formulation

For trajectory tracking, we must rely on error dynamics and relinearization about a time-varying reference $\mathbf{x}_{ref}(t)$.

### Tracking Error

Define the error state:

$$\mathbf{e} = \mathbf{x} - \mathbf{x}_{ref}$$

The reference trajectory must satisfy the dynamics (it must be physically realizable):

$$\dot{\mathbf{x}}_{ref} = f(\mathbf{x}_{ref}, \mathbf{u}_{ref})$$

where $\mathbf{u}_{ref}$ is the feedforward control, which is the input required to execute the reference trajectory.

### Linearized Error Dynamics

Taylor expanding about the reference and subtracting the reference dynamics yields:

$$\dot{\mathbf{e}} = A(\mathbf{x}_{ref}) \mathbf{e} + B \delta\mathbf{u}$$

where $\delta\mathbf{u} = \mathbf{u} - \mathbf{u}_{ref}$ is the feedback correction.

The Jacobian $A = \partial f / \partial \mathbf{x}$ evaluated at the reference state is:

$$
A = \begin{bmatrix}
0 & 0 & -\bar{v}\sin\theta_{ref} & \frac{1}{2}\cos\theta_{ref} & \frac{1}{2}\cos\theta_{ref} \\[0.5em]
0 & 0 & \bar{v}\cos\theta_{ref} & \frac{1}{2}\sin\theta_{ref} & \frac{1}{2}\sin\theta_{ref} \\[0.5em]
0 & 0 & 0 & -\frac{1}{L} & \frac{1}{L} \\[0.5em]
0 & 0 & 0 & -\frac{1}{\tau} & 0 \\[0.5em]
0 & 0 & 0 & 0 & -\frac{1}{\tau}
\end{bmatrix}
$$

where $\bar{v} = (v_{l,ref} + v_{r,ref})/2$ is the reference forward velocity.

The input Jacobian $B = \partial f / \partial \mathbf{u}$ is constant:

$$
B = \begin{bmatrix}
0 & 0 \\ 0 & 0 \\ 0 & 0 \\ \frac{1}{\tau} & 0 \\ 0 & \frac{1}{\tau}
\end{bmatrix}
$$

## Discrete-Time Dynamics

For implementation, we discretize using zero-order hold on the input. Given continuous-time system $(A_c, B_c)$ and timestep $\Delta t$:

$$A_d = I + A_c \Delta t + \frac{(A_c \Delta t)^2}{2!} + \cdots \approx e^{A_c \Delta t}$$

$$B_d = \left(\int_0^{\Delta t} e^{A_c \tau} d\tau\right) B_c \approx \Delta t \cdot B_c$$

For small $\Delta t$, the first-order approximation $A_d \approx I + A_c \Delta t$ and $B_d \approx \Delta t \cdot B_c$ is often sufficient.

The discrete-time error dynamics become:

$$\mathbf{e}_{k+1} = A_d \mathbf{e}_k + B_d \delta\mathbf{u}_k$$

## Control Law Structure

The trajectory tracking control law decomposes into feedforward and feedback:

$$\mathbf{u} = \mathbf{u}_{ref} + \delta\mathbf{u}$$

The feedforward $\mathbf{u}_{ref}$ comes from the reference trajectory (see [Reference Trajectories](notebooks/Reference%20Trajectories.ipynb)). The feedback $\delta\mathbf{u}$ corrects deviations using one of:

- **LQR**: $\delta\mathbf{u} = -K \mathbf{e}$
- **MPC**: solve optimization over prediction horizon

See [Controllers](notebooks/Controllers.ipynb) for implementation details.

## Angle Wrapping

The heading error $e_\theta = \theta - \theta_{ref}$ must be wrapped to $[-\pi, \pi]$ to avoid discontinuities:

$$e_\theta = \text{atan2}(\sin(\theta - \theta_{ref}), \cos(\theta - \theta_{ref}))$$

or equivalently:

$$e_\theta = ((\theta - \theta_{ref}) + \pi) \mod 2\pi - \pi$$

This is critical for correct controller behavior near $\theta = \pm\pi$.
