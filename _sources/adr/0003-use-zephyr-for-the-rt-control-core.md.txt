# 3. Use Zephyr for the RT control core

Date: 2025-11-23

## Status

Accepted

## Context

Following [ADR-0002](0002-split-compute-and-real-time-control-to-separate-processors.md), we have decided to split Benji's compute and real-time control tasks to separate processors. The real-time control core will be implemented on a microcontroller (MCU) that interfaces directly with sensors and actuators. There are enough tasks that the MCU will need to handle that using a real-time operating system (RTOS) is advisable, especially considering the benefits and additional features that an RTOS can provide.

A few factors are driving the choice of an RTOS:

- **Device Support** - As a solo developer, it's not realistic that I can write and maintain all the necessary device drivers while also completing the project in a reasonable time frame, and it's silly to attempt to do so. A well-supported ecosystem with existing drivers for the required peripherals is essential.
- **Update and Maintenance** - With the planned hardware design, the chip will not be as easy to get to for software updates. An RTOS that supports DFU (Device Firmware Update) over the air or through a simple interface is highly desirable, particularly one with good community support.
- **Portability** - Initially, the hardware will be hacked together to prove functionality, but the long-term plan is to migrate to a custom PCB, which may involve different MCUs, sensors, and actuator drivers. A robust abstraction layer provided by an RTOS will make this migration significantly easier.

## Decision

We will use the Zephyr Project for the RT control core. Simply put, it meets all the criteria outlined above, in a way that others do not come close to. It also brings a wealth of additional functionality, largely for free, and has a very wide range of supported hardware. It is also in line with my desired tech stack.

## Consequences

Using Zephyr will significantly reduce the development time for the RT control core, thanks to its extensive device support and active community. The abstraction layers provided by Zephyr will facilitate future hardware changes, making it easier to migrate to a custom PCB. Additionally, Zephyr's built-in support for DFU will simplify the update process for the deployed hardware. However, it's known for having a very steep learning curve (particularly for those unfamiliar with embedded Linux and its device/driver model), which may make it difficult to onboard others in the future.
