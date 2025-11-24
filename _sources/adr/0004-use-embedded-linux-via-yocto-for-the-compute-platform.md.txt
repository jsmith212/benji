# 4. Use embedded linux via Yocto for the compute platform

Date: 2025-11-23

## Status

Accepted

## Context

Following [ADR-0002](0002-split-compute-and-real-time-control-to-separate-processors.md), we have decided to split Benji's compute and real-time control tasks to separate processors. The compute platform will be implemented on an application processor (SBC) that handles compute-intensive tasks such as image processing and path planning. Realistically, there's no world in which Linux isn't the operating system in use onboard the compute platform. The choice really comes down to how to maintain the Linux system. In particular, there are a few things we value:

- **Reproducibility** - We want to be able to reproduce the exact same system image at any time in the future, whether for bug fixes, updates, or new hardware revisions. This includes not just the application software but also the underlying OS and all dependencies.
- **Customization** - We may need to customize the Linux kernel or other system components to meet specific performance or functionality requirements. A build system that allows for easy customization is essential.
- **Maintainability** - The system should be easy to maintain and update over time, with a clear process for applying updates and managing dependencies.
- **Portability** - It's not entirely clear how small/low-power of an SBC we can get away with, and we can't really know until the software is largely done. We need a system that can be easily ported to different hardware platforms as needed.

There are effectively two choices: using a pre-built Linux distribution (such as Raspberry Pi OS, Ubuntu, etc.) and customizing it as needed through scripts, or using an embedded Linux build system (such as Yocto, Buildroot, etc.) to create a custom Linux distribution tailored to our specific needs.

## Decision

We will use Yocto to build a custom embedded Linux distribution for the compute platform. Yocto meets our requirements for reproducibility, customization, and maintainability. It allows us to define the entire system configuration in a set of recipes and configuration files, enabling us to reproduce the same system image consistently. Yocto also provides a high degree of customization, allowing us to tailor the Linux kernel and other system components to our specific needs. Additionally, Yocto has a large and active community, which can provide support and resources as needed.

## Consequences

Using Yocto will require a significant initial investment in time and effort to set up the build system and create the necessary recipes and configuration files. Much of this can be reused from existing projects (i.e. AutonomOS), but there will still be at least one custom layer required for Benji-specific applications. That said, the benefits are worth it, especially since we can reuse an existing project. We can also more easily integrate an OTA update mechanism (e.g. Mender) to make the system significantly easier to maintain over time.
