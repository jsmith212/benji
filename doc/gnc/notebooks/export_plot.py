#!/usr/bin/env python3
"""Export controller comparison plot for presentation."""

import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are

from utils import angle_wrap, state_error, generate_smooth_reference_trajectory
from mpc_controller import QPMPC, NonlinearMPC, compute_jacobians_at_state

# Set dark theme for presentation
plt.style.use('dark_background')

# Robot parameters
L = 0.14
tau = 0.2
v_cruise = 0.25
u_max = 0.5
n_x, n_u = 5, 2

# Simulation
T_final = 60.0
dt = 0.05
t = np.arange(0, T_final, dt)
N = len(t)

Q_noise = np.diag([1e-5, 1e-5, 1e-6, 1e-4, 1e-4])
Q_noise_chol = np.linalg.cholesky(Q_noise)


def nonlinear_dynamics(x, u, dt_step, L, tau):
    x_pos, y_pos, theta, v_l, v_r = x
    u_l, u_r = u
    v_avg = (v_l + v_r) / 2.0
    return np.array([
        x_pos + v_avg * np.cos(theta) * dt_step,
        y_pos + v_avg * np.sin(theta) * dt_step,
        theta + (v_r - v_l) / L * dt_step,
        v_l + (-v_l + u_l) / tau * dt_step,
        v_r + (-v_r + u_r) / tau * dt_step
    ])


# Generate trajectory
waypoints = np.array([
    [0.0, 0.0],
    [3.0, 3.0],
    [6.0, 2.0],
    [8.0, 5.0],
])
x_ref, u_ref = generate_smooth_reference_trajectory(waypoints, t, v_cruise, L, tau, turn_time=1.0)

# LQR setup
Q_lqr = np.diag([20.0, 20.0, 5.0, 0.1, 0.1])
R_lqr = np.diag([1.0, 1.0])
x_nom = np.array([0, 0, 0, v_cruise, v_cruise])
A_c, B_c = compute_jacobians_at_state(x_nom, L, tau)
A_d = np.eye(n_x) + dt * A_c
B_d = dt * B_c
P_lqr = solve_discrete_are(A_d, B_d, Q_lqr, R_lqr)
K_lqr = np.linalg.solve(R_lqr + B_d.T @ P_lqr @ B_d, B_d.T @ P_lqr @ A_d)


def lqr_ff_control(x, x_ref, u_ref, K, u_max):
    e = state_error(x, x_ref)
    u = u_ref - K @ e
    return np.clip(u, -u_max, u_max)


# Simulate LQR
final_waypoint = waypoints[-1]
goal_tol = 0.1

x_lqr = np.zeros((n_x, N))
x_lqr[:, 0] = x_ref[:, 0]
rng = np.random.default_rng(42)

for k in range(N - 1):
    u = lqr_ff_control(x_lqr[:, k], x_ref[:, k], u_ref[:, k], K_lqr, u_max)
    w = Q_noise_chol @ rng.standard_normal(n_x)
    x_lqr[:, k+1] = nonlinear_dynamics(x_lqr[:, k], u, dt, L, tau) + w
    dist = np.sqrt((x_lqr[0, k+1] - final_waypoint[0])**2 + (x_lqr[1, k+1] - final_waypoint[1])**2)
    if dist < goal_tol and k > 50:
        x_lqr = x_lqr[:, :k+2]
        break

# QP-MPC
Q_mpc = np.diag([10.0, 10.0, 5.0, 0.1, 0.1])
R_mpc = np.diag([0.1, 0.1])
R_rate = np.diag([2.0, 2.0])
mpc_qp = QPMPC(L, tau, dt, Q_mpc, R_mpc, R_rate, 15, u_max, exp_weight=1.0)

x_qpmpc = np.zeros((n_x, N))
x_qpmpc[:, 0] = x_ref[:, 0]
rng = np.random.default_rng(42)
mpc_qp.reset()

for k in range(N - 1):
    k_end = min(k + 16, N)
    x_ref_h = x_ref[:, k:k_end]
    u_ref_h = u_ref[:, k:k_end]
    if x_ref_h.shape[1] < 16:
        pad = 16 - x_ref_h.shape[1]
        x_ref_h = np.hstack([x_ref_h, np.tile(x_ref[:, -1:], (1, pad))])
        u_ref_h = np.hstack([u_ref_h, np.tile(u_ref[:, -1:], (1, pad))])

    u = mpc_qp.compute_control(x_qpmpc[:, k], x_ref_h, u_ref_h)
    w = Q_noise_chol @ rng.standard_normal(n_x)
    x_qpmpc[:, k+1] = nonlinear_dynamics(x_qpmpc[:, k], u, dt, L, tau) + w

    dist = np.sqrt((x_qpmpc[0, k+1] - final_waypoint[0])**2 + (x_qpmpc[1, k+1] - final_waypoint[1])**2)
    if dist < goal_tol and k > 50:
        x_qpmpc = x_qpmpc[:, :k+2]
        break

# NL-MPC
def dynamics_for_mpc(x, u, dt_step):
    return nonlinear_dynamics(x, u, dt_step, L, tau)

Q_nl = np.diag([10.0, 10.0, 5.0, 0.1, 0.1])
R_nl = np.diag([0.1, 0.1])
mpc_nl = NonlinearMPC(dynamics_for_mpc, dt, Q_nl, R_nl, 10, 5, u_max)

x_nlmpc = np.zeros((n_x, N))
x_nlmpc[:, 0] = x_ref[:, 0]
rng = np.random.default_rng(42)
mpc_nl.reset()

print("Running NL-MPC (slow)...")
for k in range(N - 1):
    k_end = min(k + 11, N)
    x_ref_h = x_ref[:, k:k_end]
    u_ref_h = u_ref[:, k:k_end]
    if x_ref_h.shape[1] < 11:
        pad = 11 - x_ref_h.shape[1]
        x_ref_h = np.hstack([x_ref_h, np.tile(x_ref[:, -1:], (1, pad))])
        u_ref_h = np.hstack([u_ref_h, np.tile(u_ref[:, -1:], (1, pad))])

    u = mpc_nl.compute_control(x_nlmpc[:, k], x_ref_h, u_ref_h)
    w = Q_noise_chol @ rng.standard_normal(n_x)
    x_nlmpc[:, k+1] = nonlinear_dynamics(x_nlmpc[:, k], u, dt, L, tau) + w

    dist = np.sqrt((x_nlmpc[0, k+1] - final_waypoint[0])**2 + (x_nlmpc[1, k+1] - final_waypoint[1])**2)
    if dist < goal_tol and k > 50:
        x_nlmpc = x_nlmpc[:, :k+2]
        break

# Compute RMS position errors
# Need to match lengths with reference
n_lqr = x_lqr.shape[1]
n_qpmpc = x_qpmpc.shape[1]
n_nlmpc = x_nlmpc.shape[1]

err_lqr = np.sqrt((x_lqr[0, :] - x_ref[0, :n_lqr])**2 + (x_lqr[1, :] - x_ref[1, :n_lqr])**2)
err_qpmpc = np.sqrt((x_qpmpc[0, :] - x_ref[0, :n_qpmpc])**2 + (x_qpmpc[1, :] - x_ref[1, :n_qpmpc])**2)
err_nlmpc = np.sqrt((x_nlmpc[0, :] - x_ref[0, :n_nlmpc])**2 + (x_nlmpc[1, :] - x_ref[1, :n_nlmpc])**2)

rms_lqr = np.sqrt(np.mean(err_lqr**2)) * 100  # cm
rms_qpmpc = np.sqrt(np.mean(err_qpmpc**2)) * 100  # cm
rms_nlmpc = np.sqrt(np.mean(err_nlmpc**2)) * 100  # cm

print(f"RMS Position Error:")
print(f"  LQR+FF:  {rms_lqr:.1f} cm")
print(f"  QP-MPC:  {rms_qpmpc:.1f} cm")
print(f"  NL-MPC:  {rms_nlmpc:.1f} cm")

# Create plot
fig, ax = plt.subplots(figsize=(8, 6), facecolor='#0f0f0f')
ax.set_facecolor('#0f0f0f')

ax.plot(x_ref[0, :], x_ref[1, :], 'w--', linewidth=2, label='Reference', alpha=0.5)
ax.plot(x_lqr[0, :], x_lqr[1, :], '#facc15', linewidth=2, label=f'LQR+FF ({rms_lqr:.1f}cm)')
ax.plot(x_qpmpc[0, :], x_qpmpc[1, :], '#f87171', linewidth=2, label=f'QP-MPC ({rms_qpmpc:.1f}cm)')
ax.plot(x_nlmpc[0, :], x_nlmpc[1, :], '#4ade80', linewidth=2, label=f'NL-MPC ({rms_nlmpc:.1f}cm)')
ax.plot(waypoints[:, 0], waypoints[:, 1], 'wo', markersize=10)

ax.set_xlabel('x [m]', fontsize=14)
ax.set_ylabel('y [m]', fontsize=14)
ax.set_aspect('equal')
ax.grid(True, alpha=0.2)
ax.legend(fontsize=12, loc='upper left')
ax.tick_params(labelsize=12)

plt.tight_layout()
plt.savefig('/home/smitj/Git/benji/doc/overview-pres/public/assets/gnc_comparison.png', dpi=150, facecolor='#0f0f0f', edgecolor='none')
print("Saved to /home/smitj/Git/benji/doc/overview-pres/public/assets/gnc_comparison.png")
plt.close()
