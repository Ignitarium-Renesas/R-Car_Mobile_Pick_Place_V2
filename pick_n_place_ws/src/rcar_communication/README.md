# ROS2 Package: rcar_communication

The `rcar_communication` package facilitates communication and control between the Renesas R-Car board, a robotic arm, and a mobile platform. It is designed for a demonstration setup to showcase an autonomous mobile pick-and-place operation using ROS 2.

## Features
- Communication between the R-Car board, robotic arm, and mobile platform.
- Execution of pick-and-place tasks autonomously.
- Support for search and pick operations using ROS 2 services.

## Prerequisites
- ROS 2 Humble or later.
- Network access to the robotic arm controller and mobile base.
- Pre-configured R-Car board with Docker and ROS 2 workspace (`rcar_ws`).

## Building the Package
1. Clone the repository:
   ```bash
   https://gitlab.ignitarium.in/ign-ai/customer/renesas/rcar/mycobot280.git
   
   ```

2. Build the workspace:
   ```bash
   colcon build
   ```

3. Source the setup file:
   ```bash
   source install/setup.bash
   ```

## Run the services

1. Launch the realsense D405 camera module, log in to the Mecharm using SSH:
```
ssh er@192.168.0.146
# Password: Elephant (default password for Elephant Robotics Hardware)
```
2. Launch the RealSense Camera Node
Open a terminal in the Mecharm and launch the camera node:
```
ros2 launch realsense2_camera rs_launch.py
```
3. Launch the Arm Server Script
Open a terminal in the mecharm and launch the camera node:
```
python3 Server_280.py
```
4. Follow the [readme](../pose_estimation_pkg/README.md) and run the pose estimation node.

5. Run the services

```
ros2 launch rcar_communication pick_service.launch.py
```