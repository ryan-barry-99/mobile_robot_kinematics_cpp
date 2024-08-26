# Mobile Robot Kinematic Library

## Overview

This project consists of a C++ library for handling kinematic calculations for mobile robots and robotic arms. The library is a port from [Python](https://github.com/ryan-barry-99/mobile-robot-kinematics) to C++ and aims to provide efficient and accurate computations for robot kinematics. The library includes functionality for both forward and inverse kinematics, as well as handling wheel properties and transformations.

## Components

The library is organized into several key classes:

- **`Wheel`**: Represents a single wheel in the robot's kinematic system. Contains properties and methods for calculating the Jacobian and Coriolis matrices.

- **`MobileRobotKinematics`**: Handles the kinematic calculations for a mobile robot, including the forward and inverse kinematics. Updates rotation and recalculates matrices based on the wheel configurations.

- **`MobileRobot`**: Manages a collection of `Wheel` objects and updates the kinematics model when wheels are added or removed.

## Installation

### Prerequisites

- **C++ Compiler**: Ensure you have a C++ compiler installed (e.g., g++, clang++).
- **Eigen Library**: This library uses the Eigen library for matrix operations. Install Eigen via package manager or download it from [Eigen's official site](https://eigen.tuxfamily.org/dox/GettingStarted.html).

