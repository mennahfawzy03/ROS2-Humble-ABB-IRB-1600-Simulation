# Manipulator Package Documentation

This documentation provides instructions on how to use and continue developing the `manipulator` package for launching a simulation in RVIZ.

## Table of Contents

- [Dependencies Installation](#dependencies-installation)
- [Workspace Setup](#workspace-setup)
- [Clone the Manipulator Package](#clone-the-manipulator-package)
- [Build the Workspace](#build-the-workspace)
- [Source the Setup File](#source-the-setup-file)
- [Launch the Simulation](#launch-the-simulation)
- [Future Work](#future-work)

## Dependencies Installation

To install the necessary ROS2 packages, run the following commands:

```bash
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-rviz
```

## Workspace Setup

Create the workspace by running the following commands:

```bash
# Navigate to your preferred workspace directory
mkdir -p ~/ws_manipulator/src
cd ~/ws_manipulator
```

## Clone the Manipulator Package

Navigate to the `src` directory of the workspace and clone the `manipulator` package:

```bash
cd ~/ws_manipulator/src
git clone https://github.com/yourusername/manipulator.git
```

## Build the Workspace

After cloning the package, build the workspace:

```bash
cd ~/ws_manipulator
colcon build
```

## Source the Setup File

Source the setup file to overlay this workspace on your environment:

```bash
source ~/ws_manipulator/install/setup.bash
```

## Launch the Simulation

To launch the robot in RVIZ and run the Joint State Publisher to move the robot joints, use the following command:

```bash
ros2 launch manipulator display.launch.py
```

## Future Work

Here are some suggested areas for future work on this package:

- **Add More Joints and Links**: Extend the URDF file to include more joints and links for a more complex robot model.
- **Improve RVIZ Visuals**: Enhance the RVIZ configuration for better visualization of the robot and its environment.
- **Integrate with Gazebo**: Create launch files and configurations to simulate the robot in Gazebo.
- **Develop Control Algorithms**: Implement and test various control algorithms for the robot.
- **Documentation and Tutorials**: Expand the documentation with detailed tutorials on extending and using the package.

---

By following these steps, you will be able to set up, use, and continue developing the `manipulator` package for simulating the ABB IRB 1600 robot in RVIZ. If you have any questions or need further assistance, feel free to reach out or check the repository's issues and discussion sections.
