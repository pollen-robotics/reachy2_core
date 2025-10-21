# Reachy2 Core

This repository provides the core ROS 2 packages required to simulate, configure, and launch the **Reachy 2** humanoid robot. It includes URDF descriptions, launch files, Gazebo simulation assets, and control interfaces used across real and simulated platforms.

## Overview

The repo is structured as a multi-package ROS 2 workspace and serves as the foundation for developing and testing Reachy 2 behaviors, whether in hardware or in simulation.

### Included Packages

- **`reachy_bringup`**  
  Launch files and runtime orchestration for both real and simulated deployments.

- **`reachy_config`**  
  YAML-based configuration files for Reachy’s kinematics, dynamics, and URDF parameterization.

- **`reachy_controllers`**  
  ROS 2 control nodes for interfacing with Reachy hardware and simulated joints.

- **`reachy_description`**  
  Robot description files (URDF, meshes, xacro) for Reachy 2, used across simulation and visualization.

- **`reachy_fake`**  
  Fake interfaces for mimicking joint states and simulating Reachy's response without hardware.

- **`reachy_gazebo`**  
  Gazebo simulation environment for Reachy 2, including plugins and world integration.

- **`reachy_gazebo_gripper_glue`**  
  Bridges and constraints for integrating the gripper with Gazebo’s physics and control pipeline.

- **`reachy_utils`**  
  Common utility functions and shared tools used by multiple packages.

## Dependencies

Install ROS 2 and the dependencies listed in `requirements.txt` and `pyproject.toml`. You can also use the included `dependencies.sh` script to install system-wide dependencies required for simulation and control.

## License

This project is licensed under the **Apache License 2.0** – see the `LICENSE` file for details.