# State Estimation

This page describes the Extended Kalman Filter (EKF) used for state estimation, including the sensor models and fusion approach.

## The Estimation Problem

The robot state $\mathbf{x} = [x, y, \theta, v_l, v_r]^T$ cannot be measured directly. We have:

- **IMU**: Provides heading $\theta$ (magnetometer), angular rate $\dot{\theta}$ (gyroscope), and body-frame accelerations $a_x, a_y$ (accelerometer)
- **Wheel encoders**: Provide wheel velocities $v_l$, $v_r$
- **Camera (low rate)**: Provides intermittent position estimates $(x, y)$ at ~1-5 Hz

Without direct position measurements, the filter must integrate velocity and heading to estimate position—this is **dead reckoning**, which drifts over time. The accelerometer helps by providing additional velocity information, while camera measurements periodically correct accumulated drift.

## Extended Kalman Filter

The EKF handles nonlinear dynamics by linearizing about the current estimate at each timestep.

### Predict Step

Given the previous estimate $\hat{\mathbf{x}}_{k-1|k-1}$ and covariance $P_{k-1|k-1}$, propagate using the nonlinear dynamics:

$$\hat{\mathbf{x}}_{k|k-1} = f(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_{k-1})$$

Compute the Jacobian at the current estimate:

$$F_k = \left.\frac{\partial f}{\partial \mathbf{x}}\right|_{\hat{\mathbf{x}}_{k-1|k-1}}$$

Propagate the covariance:

$$P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q$$

where $Q$ is the process noise covariance.

### Update Step

Given measurement $\mathbf{y}_k$ with measurement model $h(\mathbf{x})$:

**Innovation:**

$$\tilde{\mathbf{y}}_k = \mathbf{y}_k - h(\hat{\mathbf{x}}_{k|k-1})$$

**Kalman gain:**

$$K_k = P_{k|k-1} H_k^T (H_k P_{k|k-1} H_k^T + R)^{-1}$$

where $H_k = \partial h / \partial \mathbf{x}$ is the measurement Jacobian and $R$ is the measurement noise covariance.

**State update:**

$$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + K_k \tilde{\mathbf{y}}_k$$

**Covariance update:**

$$P_{k|k} = (I - K_k H_k) P_{k|k-1}$$

## Sensor Models

### IMU + Encoders (Baseline)

The baseline measurement vector is:

$$\mathbf{y} = \begin{bmatrix} \theta \\ \dot{\theta} \\ v_l \\ v_r \end{bmatrix} = \begin{bmatrix} \theta \\ (v_r - v_l)/L \\ v_l \\ v_r \end{bmatrix}$$

The measurement matrix (linear in state):

$$
H = \begin{bmatrix}
0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & -1/L & 1/L \\
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$

Position $(x, y)$ is not directly measured. The filter must infer position from velocity integration, leading to unbounded uncertainty growth.

### Adding the Accelerometer

The accelerometer measures body-frame linear accelerations $(a_x^b, a_y^b)$. For a tank drive robot, these relate to the state as:

$$a_x^b = \dot{v}_{avg} = \frac{\dot{v}_l + \dot{v}_r}{2} = \frac{1}{2\tau}\left((u_l - v_l) + (u_r - v_r)\right)$$
$$a_y^b = v_{avg} \cdot \dot{\theta} = \frac{v_l + v_r}{2} \cdot \frac{v_r - v_l}{L}$$

The centripetal acceleration $a_y^b$ arises from the robot turning. With the accelerometer, the measurement vector becomes:

$$\mathbf{y}_{accel} = \begin{bmatrix} \theta \\ \dot{\theta} \\ v_l \\ v_r \\ a_x^b \\ a_y^b \end{bmatrix}$$

The measurement is **nonlinear** in the state due to the centripetal term. The measurement Jacobian is:

$$
H_{accel} = \begin{bmatrix}
0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & -1/L & 1/L \\
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & -1/(2\tau) & -1/(2\tau) \\
0 & 0 & 0 & \frac{\partial a_y^b}{\partial v_l} & \frac{\partial a_y^b}{\partial v_r}
\end{bmatrix}
$$

where the centripetal acceleration partial derivatives are:

$$\frac{\partial a_y^b}{\partial v_l} = \frac{1}{2L}(v_r - 2v_l), \quad \frac{\partial a_y^b}{\partial v_r} = \frac{1}{2L}(2v_r - v_l)$$

**Benefit:** The accelerometer provides redundant velocity information. When wheel slip occurs (encoders report incorrect velocities), the accelerometer can detect the discrepancy.

### With Camera Position Measurements

When camera position measurements are available (typically at lower rate, ~1-5 Hz):

$$\mathbf{y}_{camera} = \begin{bmatrix} x \\ y \end{bmatrix}$$

$$
H_{camera} = \begin{bmatrix}
1 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0
\end{bmatrix}
$$

Position measurements bound the uncertainty, preventing drift.

### Multi-Rate Fusion

Different sensors operate at different rates:

| Sensor            | Typical Rate | Latency   |
| ----------------- | ------------ | --------- |
| IMU (gyro, accel) | 100-400 Hz   | < 1 ms    |
| Wheel encoders    | 20-100 Hz    | < 1 ms    |
| Camera position   | 1-10 Hz      | 50-200 ms |

The EKF naturally handles multi-rate measurements:

1. **Predict** at the highest rate (every control cycle, typically 20-50 Hz)
2. **Update** with IMU/encoder data at their native rate
3. **Update** with camera data when available (asynchronous)

Each update uses the appropriate measurement model ($H$, $R$ matrices). See the [Multi-Rate Sensor Fusion](notebooks/Multi-Rate%20Sensor%20Fusion.ipynb) notebook for implementation details.

## Noise Covariances

### Process Noise $Q$

Process noise accounts for unmodeled dynamics and discretization errors. Typical structure:

$$Q = \text{diag}(\sigma_x^2, \sigma_y^2, \sigma_\theta^2, \sigma_{v_l}^2, \sigma_{v_r}^2)$$

where:

- Position noise $\sigma_x, \sigma_y \sim 10^{-5}$ m² (small, dynamics are well-modeled)
- Heading noise $\sigma_\theta \sim 10^{-6}$ rad²
- Velocity noise $\sigma_{v_l}, \sigma_{v_r} \sim 10^{-4}$ (m/s)² (motor disturbances)

### Measurement Noise $R$

Measurement noise reflects sensor accuracy:

| Sensor           | Typical $\sigma$ | Notes                            |
| ---------------- | ---------------- | -------------------------------- |
| Magnetometer (θ) | 0.03 rad (~2°)   | Subject to magnetic interference |
| Gyroscope (θ̇)    | 0.01 rad/s       | Bias drift over time             |
| Accelerometer    | 0.1 m/s²         | Bias, vibration-sensitive        |
| Wheel encoders   | 0.03 m/s         | Slip, quantization               |
| Camera (x, y)    | 0.05 m           | Environment-dependent, latency   |

## Position Uncertainty Growth

Without position measurements, the EKF's position uncertainty grows without bound. The growth rate depends on:

1. **Heading uncertainty**: Small heading errors compound into large position errors over distance traveled
2. **Velocity uncertainty**: Integrates directly into position
3. **Process noise**: Adds uncertainty each timestep

For indoor navigation, position measurements every few seconds are typically sufficient to bound uncertainty to acceptable levels.

## Implementation Notes

### Angle Wrapping in Innovation

The heading innovation must be wrapped:

$$\tilde{\theta} = \text{wrap}(\theta_{measured} - \theta_{predicted})$$

### Numerical Stability

The Joseph form of the covariance update is more numerically stable:

$$P_{k|k} = (I - K_k H_k) P_{k|k-1} (I - K_k H_k)^T + K_k R K_k^T$$

### Observability

The system is **not observable** from IMU/encoder measurements alone—position is unobservable. The filter will estimate velocity and heading accurately, but position will drift. Adding even occasional position measurements makes the full state observable.
