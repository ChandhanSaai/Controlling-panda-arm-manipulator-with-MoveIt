# MoveIt-Based Robot Arm Manipulation

This project implements a pick-and-place task using the Panda robotic arm with **MoveIt** in **ROS 2**. It leverages predefined target states and trajectory planning for seamless robotic arm control.

## Features
- **End-Effector Pose Transitions:** Smooth transitions between predefined states for efficient manipulation tasks.
- **Predefined Target States:**
  - `Open Gripper` for object release.
  - `Closed Gripper` for object grasping.
  - `Home Position` for resetting the arm.
- **Collision-Free Planning:** Ensures safe and precise trajectory execution.

## Technologies and Libraries
- **Frameworks:** ROS 2, MoveIt
- **Programming Language:** C++
- **Libraries Used:**
  ```cpp
  #include "rclcpp/rclcpp.hpp"
  #include "moveit/move_group_interface/move_group_interface.h"
  #include <thread>
  #include <chrono>
  #include <vector>
  ```
## Prerequisites
- Install ROS 2 (Rolling or Galactic recommended)
- Install MoveIt for ROS 2
- Set up the Panda robotic arm in the MoveIt environment.
## Setup Instructions
- Clone the repository into your ROS 2 workspace:
```cpp
git clone https://github.com/ChandhanSaai/Controlling-panda-arm-manipulator-with-MoveIt.git
```
- Build and Source the workspace:
```cpp
colcon build
source install/setup.bash
```
## Running the Project
- Launch RViz with the Panda Robot
```cpp
ros2 launch chandhan_panda demo.launch.py
```
- Run the Pick-and-Place Script
```cpp
ros2 run package_120387364 script
```
## Project Details
- The project uses pre-programmed in the MoveIt config settings for gripper states (open, closed) and a home position.
- Implements collision-free trajectory planning with MoveIt.
- Suitable for pick-and-place tasks in simulation or real-world robotic setups.
