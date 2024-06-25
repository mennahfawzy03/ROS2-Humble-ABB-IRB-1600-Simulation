This is a documentation for how to use the `gazebo_test` package in the `ws_gazebo` workspace.

### ROS2 Packages Installation

To install the necessary dependencies, run the following commands:

```bash
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros-core
sudo apt install ros-humble-geometry2

## Clone the gazebo test Package

Navigate to the `src` directory of the workspace and clone the `manipulator` package:

```bash
cd ~/ws_manipulator/src
git clone https://github.com/yourusername/manipulator.git
```

## Build the Workspace

After cloning the package, build the workspace:

```bash
cd ~/ws_gazebo
colcon build
```

## Source the Setup File

Source the setup file to overlay this workspace on your environment:

```bash
source ~/ws_gazebo/install/setup.bash
```

## Launch the Simulation

To launch the robot in RVIZ and run the Joint State Publisher to move the robot joints, use the following command:

```bash
ros2 launch manipulator display.launch.py
```
