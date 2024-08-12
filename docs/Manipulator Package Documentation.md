# Manipulator Package Documentation

This documentation provides instructions on how to use and continue developing the `manipulator` package for launching a simulation in RVIZ.

## ROS2 Packages Installation

Ensure that all existing packages are up to date by running the following commands:
```bash
sudo apt update
```
```bash
sudo apt upgrade
```

To install the necessary ROS2 packages, run the following commands:

```bash
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-rviz2
```

## Workspace Setup

Create the workspace by running the following commands:

```bash
mkdir -p ~/ws_manipulator/src
```

## Clone the Manipulator Package

Navigate to the `src` directory of the workspace if not already in it and clone the `manipulator` package:
```bash
cd ws_manipulator/src
git clone https://github.com/mennahfawzy03/ROS2_abb_irb_1600.git
```
Next, to remove all unwanted components of the whole package and isolate the `manipulator` package, open your Files app, go into the `ws_manipulator/src/ROS2_abb_irb_1600` folder and drag the `manipulator` folder into the main `src` directory. Lastly delete the `ROS2_abb_irb_1600` folder so that all that remains is the `manipulator` package within your `src` directory.

## Build the Workspace

After cloning the package, build the workspace:

```bash
cd ws_manipulator
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

- **Add End Effector to URDF**: Include the nozzle in the URDF for the robot geometry modeling in RVIZ and Gazebo.
- **Improve Joint and Link Locations**: Update the origin tags in the URDF model by checking the required translations in the x, y, and z axes from Blender to enhance the RVIZ configuration.
- **Kinematics**: Once the origins and links are correctly defined, you can proceed with moving the sliders from the joint state publisher in RVIZ or by writing a script and launching it in the gazebo_test package for joint planning.
- **Integrate with Gazebo**: Further develop the launch and xacro files in the URDF to simulate the robot in Gazebo. Check the initial gazebo_test package in this repository.
- **Develop Control Algorithms**: For control setup, you can use this repository as a guide: [robotic_arm_environment](https://github.com/dvalenciar/robotic_arm_environment/blob/main/). You can update the irb6640 package from [ros2_simulations](https://github.com/IFRA-Cranfield/ros2_RobotSimulation.git).
- **Develop Fluids Simulation**: Create a simulation of fluid flow using fluid simulation or deformable bodies. Different plugins to download are available.
- **Documentation and Tutorials**: Expand the documentation with detailed tutorials on extending and using the package.

---

By following these steps, you will be able to set up, use, and continue developing the `manipulator` package for simulating the ABB IRB 1600 robot in RVIZ. 
