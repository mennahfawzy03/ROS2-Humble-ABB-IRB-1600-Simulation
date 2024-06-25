
### Prerequisites

This table outlines the essential hardware and software requirements for setting up and operating ROS 2, Gazebo, and RViz for running the packages.

| **Requirement Type** | **Description**                      | **Details/Version**              |
|----------------------|--------------------------------------|----------------------------------|
| Hardware             | Computer Specifications              | Lenovo Legion S7 16IAH7          |
|                      | Memory                               | 24.0 GiB                         |
|                      | Processor                            | 12th Gen Intel® Core™ i7-12700H × 20 |
|                      | Graphics                             | NVIDIA GeForce RTX 3070 Mobile   |
| Operating System     | Supported OS                         | Ubuntu 22.04.4 LTS               |
| Software             | ROS 2 Distribution                   | Humble Hawksbill                 |
|                      | Gazebo                               | Gazebo 11                        |
|                      | Visual Studio Code                   | Latest Version                   |
| Dependencies         | Development Tools and Libraries      | Python 3.8, CMake, Git           |

# ROS2_abb_irb_1600 Repository About

This repository contains the work I have done so far to create a digital twin for 3D concrete printing. I believe it provides a solid foundation for someone to continue developing the packages in this repository which includes my progress, resources, and documentation that can serve as a quick guide for further development which should integrate fluid simulation, body kinematics, inverse kinematics for our robot, cameras, and sensor data integration.
# Table of Contents
1. [Installations](#installations)
2. [Preparing URDF](#preparing-urdf)
3. [Manipulator Package](#manipulator-package)
4. [Gazebo Test](#gazebo-test)
5. [Future Work](#future-work)
## Installations
Installation instructions for the softwares used can be found in [installations](docs/installations.md).
## Preparing URDF
Instructions for preparing URDF for the ABB IRB 1600 can be found [here](docs/preparing_URDF.md).
## Manipulator Package
The package to simulate the ABB IRB 16000 robot in RVIZ can be found in [manipulator](manipulator/) and the documentation for the RVIZ setup is found [readme_rviz.md](docs/readme_rviz.md)
## Gazebo Test Package
The gazebo test package can be found in [gazebo_test](gazebo_test/) documentation of the Gazebo setup can be found [here](docs/readme_gazebo.md). 
## Future Work
Here are some suggested areas for future work on this package:
- **Add End Effector to URDF**: Include the nozzle in the URDF for the robot geometry modeling in RVIZ and Gazebo.
- **Improve Joint and Link Locations**: Update the origin tags in the URDF model by checking the required translations in the x, y, and z axes from Blender to enhance the RVIZ configuration.
- **Kinematics**: Once the origins and links are correctly defined, you can proceed with moving the sliders from the joint state publisher in RVIZ or by writing a script and launching it in the gazebo_test package for joint planning.
- **Integrate with Gazebo**: Further develop the launch and xacro files in the URDF to simulate the robot in Gazebo. Check the initial gazebo_test package in this repository.
- **Develop Control Algorithms**: For control setup, you can use this repository as a guide: [robotic_arm_environment](https://github.com/dvalenciar/robotic_arm_environment/blob/main/). You can update the irb6640 package from [ros2_simulations](https://github.com/IFRA-Cranfield/ros2_RobotSimulation.git).
- **Develop Fluids Simulation**: Create a simulation of fluid flow using fluid simulation or deformable bodies. Different plugins to download are available.
- **Documentation and Tutorials**: Expand the documentation with detailed tutorials on extending and using the package.

## Specifications
## Ros2 installation 
- sourcing ros2
- testing if its fuinctional
- ROS2 documentation: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
### Testing the turtlesim node in ros2
-  ros2 run turtlesim turtlesim_node
- then in a new terminal, ros2 run turtlesim turtle_teleop_key 
## Gazebo installation 
- checking the version
### Restarting Gazebo
-ps aux | grep gz

## Create a workspace 
- sourcing the workspace 
## Create a package 

```
colcon build
```

## Converting package that was made in ros1 to one that is compatible with ros2
https://industrial-training-master.readthedocs.io/en/humble/_source/session7/ROS1-to-ROS2-porting.html

# Trouble shooting ros2 and gazebo11
```
sudo apt install ros-humble-gazebo-ros-pkgs ros-
```
- the output was 'E: Unable to locate package ros-humble-gazebo-ros-control'
```
sudo apt update
```
- to find the packages related to ros-humble-gazebo
```
 apt-cache search ros-humble-gazebo
```
- there was a spelling error should have been as follows: 
```
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```
# Upgrade/Update
```
sudo apt update
```

sudo apt upgrade

# ROS2_abb_irb_1600 Repository About

Welcome to the documentation for the Digital Twin Creation project. This repository contains detailed instructions and guides on how to create a digital twin using ROS2, Gazebo, and a depth camera.

# Table of Contents

1. [Manipulator Package](#manipulator-package)
2. [Gazebo Test](#gazebo-test)
3. [Installations](#installations)
4. [Readme Gazebo](#readme-gazebo)
5. [Readme RViz](#readme-rviz)
6. [Preparing URDF](#preparing-urdf)



