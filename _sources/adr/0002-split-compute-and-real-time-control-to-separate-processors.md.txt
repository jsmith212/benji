# 2. Split compute and real time control to separate processors

Date: 2025-11-23

## Status

Accepted

## Context

Benji's capabilities can be broadly divided into two categories: compute-intensive tasks (such as image processing and path planning) and real-time control tasks (such as motor control and sensor data acquisition). The sensor and actuator list is currently baselined as follows:

- Sensors:
  - Camera (for image processing)
  - 6-axis IMU (accelerometer and gyro)
  - 3-axis magnetometer
  - Linear ToF distance sensor
  - 4x quadrature encoders (for wheel odometry)

- Actuators:
  - 4x DC gearmotors with individual drivers
  - 1x servo motor (for camera tilt)

Typically, in the interest of reducing complexity and cost, we'd prefer to run all tasks on a single application processor. Low-latency real-time tasks can be pinned to specific cores, and the Linux PREEMPT-RT kernel patch might be applied. However, it turns out it's not easy to find a specific board with adequate I/O capabilities without turning to a custom board, which is outside our capabilities to design. Splitting the I/O and low-latency control tasks to a microcontroller will still benefit from a custom board, but the complexity is significantly lower, and it's quite straightforward to find an MCU with the I/O and compute capabilities required. For compute, this significantly widens the range of suitable off-the-shelf SBCs, in particular allowing us to leverage the Raspberry Pi ecosystem.

## Decision

We will split Benji's compute and real-time control tasks to separate processors. A microcontroller (MCU) will handle real-time control tasks, interfacing directly with sensors and actuators. An application processor (SBC) will handle compute-intensive tasks, communicating with the MCU over high-bandwidth serial.

## Consequences

This allows much of the real-time control complexity to be deferred to the MCU, which can typically make use of hardware peripherals (particularly the quadrature decoders, which is difficult to implement accurately without a dedicated hardware timer). The MCU can also leverage other ecosystems that have most of the drivers required. The SBC can then focus all its resources on compute tasks, and there is no need to worry about a RT kernel.
