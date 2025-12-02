# Guidance, Navigation, and Control

This section documents the GNC algorithms for the Benji rover, a differential-drive (tank drive) mobile robot. The approach relies on error dynamics linearization about reference trajectories, enabling the use of linear control techniques for trajectory tracking.

## System Overview

Benji uses a 5-state model capturing position, heading, and wheel velocities with first-order motor dynamics:

$$\mathbf{x} = \begin{bmatrix} x & y & \theta & v_l & v_r \end{bmatrix}^T$$

The control input is commanded wheel velocities: $\mathbf{u} = \begin{bmatrix} u_l & u_r \end{bmatrix}^T$

## Notation

| Symbol             | Description                                      | Unit |
| ------------------ | ------------------------------------------------ | ---- |
| $L$                | Wheelbase (wheel-to-wheel distance)              | m    |
| $\tau$             | Motor/drivetrain time constant                   | s    |
| $x, y$             | Position in global frame                         | m    |
| $\theta$           | Heading angle                                    | rad  |
| $v_l, v_r$         | Left/right wheel velocities                      | m/s  |
| $u_l, u_r$         | Left/right wheel commands                        | m/s  |
| $\mathbf{x}_{ref}$ | Reference state trajectory                       | —    |
| $\mathbf{u}_{ref}$ | Feedforward control                              | m/s  |
| $\mathbf{e}$       | Tracking error $(\mathbf{x} - \mathbf{x}_{ref})$ | —    |

## Control Architecture

The trajectory tracking control law takes the form:

$$\mathbf{u} = \mathbf{u}_{ref} + \mathbf{u}_{fb}$$

where $\mathbf{u}_{ref}$ is the feedforward term computed from the reference trajectory, and $\mathbf{u}_{fb}$ is the feedback term that corrects deviations. Three feedback strategies are implemented:

| Controller | Feedback Law                     | Solve Time | Best For                       |
| ---------- | -------------------------------- | ---------- | ------------------------------ |
| **LQR+FF** | $\mathbf{u}_{fb} = -K\mathbf{e}$ | <0.1 ms    | Real-time, smooth trajectories |
| **QP-MPC** | Receding horizon QP (OSQP)       | ~2 ms      | Constraints, embeddable        |
| **NL-MPC** | Nonlinear optimization           | ~20 ms     | Maximum accuracy               |

## Theory

```{toctree}
---
maxdepth: 1
---
dynamics
state-estimation
controllers
```

## Implementation

```{toctree}
---
maxdepth: 1
---
notebooks/Reference Trajectories
notebooks/Controllers
notebooks/Multi-Rate Sensor Fusion
```
