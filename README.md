
# Robot Automation System for Silicone Workpiece Testing

This repository contains the software packages and control nodes for an automated robot testing system designed to evaluate silicone workpieces. The system utilizes ROS (Robot Operating System) and MoveIt to plan and execute precise multi-point trajectories for automated contact testing.

## Overview

The primary objective of this project is to perform automated surface testing on a silicone workpiece using a robotic arm Yaskawa Motomini. The system generates a dynamic zig-zag grid over the silicone surface and executes a series of stable Point-to-Point (PTP) and Cartesian contact paths to simulate stamping/pressing cycles at specified angles.

### Key Features
- **Dynamic Grid Generation**: Automatically computes a multi-point zig-zag path matching the physical dimensions of the silicone workpiece.
- **Multi-Angle Configuration**: Supports scanning profiles with customizable tilt and azimuth orientations.
- **Contact Cycle Execution**: Sequentially moves to a "hover" position above each target location before executing a controlled "stamp" operation down onto the workpiece.
- **Safe Retreat Profiles**: Implements automated lifting maneuvers between test cycles to prevent collision or damage to the surface.

---

## Core Source File

The current primary execution node for this system is located at:
`src/my_robot_controller/point_multi.cpp`

### File Breakdown: `point_multi.cpp`
This C++ node establishes the core control logic using the MoveIt `MoveGroupInterface`. It handles:
1. **`generateZigZagGrid()`**: Generates a standard $16\text{mm} \times 24\text{mm}$ mesh with an $8\text{mm}$ step size, maintaining an operational safety height ($Z = 5\text{mm}$) relative to the workpiece center workspace alignment.
2. **`computeTCPPose()`**: Calculates the Tool Center Point (TCP) orientation by translating custom tilt and azimuth angle constraints into standard ROS quaternions.
3. **`executeStablePTP()`**: Performs standard joint-space trajectories to position the end-effector safely at the hover state.
4. **`executeCartesianStamp()`**: Performs a precision-controlled straight-line Cartesian path to move from the hover threshold to the exact pressing depth.

---

## Prerequisites & Dependencies

Ensure your environment is configured with the following frameworks before compilation:
- **Ubuntu 20.04 LTS**
- **ROS Noetic Ninjemys**
- **MoveIt Motion Planning Framework**
- `tf2_geometry_msgs` & standard ROS robot perception packages

---

## Installation & Build

Clone this repository into the source space of your Catkin workspace, then compile the workspace:

```bash
# Navigate to your catkin workspace
cd ~/catkin_ws/src

# Build the package from the workspace root
cd ~/catkin_ws
catkin_make

# Source the setup file
source devel/setup.bash

```


## How to Run
1. **`Launch the Simulation/Robot Environment`**:
Ensure your specific robot description (URDF) and MoveIt configuration packages are running. For instance, if using a simulation setup:

```bash
    roslaunch my_robot_moveit_config demo.launch
```
2. **`Execute the Multi-Point Testing Node`**:
In a new terminal window, source the workspace environment and execute the primary testing binary compiled from point_multi.cpp:

```bash
   source ~/catkin_ws/devel/setup.bash
   rosrun my_robot_controller point_multi_node
