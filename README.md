
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

## Installation Instructions

### ROS 2 Humble Installation on Ubuntu 22.04
```bash
sudo apt update && sudo apt upgrade
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash

# Install additional dependencies
sudo apt install python3-colcon-common-extensions
sudo apt install ros-dev-tools

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
```
sudo apt upgrade
```
# Digital Twin Creation Documentation

Welcome to the documentation for the Digital Twin Creation project. This repository contains detailed instructions and guides on how to create a digital twin using ROS2, Gazebo, and a depth camera.

## Table of Contents
1. [Software Installations](docs/installations.md)
2. [Project Overview](docs/project_overview.md)
3. [Prerequisites](docs/prerequisites.md)
4. [Tools and Technologies](docs/tools_and_technologies.md)
5. [Setup](docs/setup.md)
6. [Creating the Digital Twin](docs/creating_the_digital_twin.md)
    - [Step 1: Model Preparation](docs/step_1_model_preparation.md)
    - [Step 2: Environment Setup](docs/step_2_environment_setup.md)
    - [Step 3: Integration](docs/step_3_integration.md)
    - [Step 4: Testing](docs/step_4_testing.md)
7. [Challenges and Solutions](docs/challenges_and_solutions.md)
8. [Future Work](docs/future_work.md)
9. [Conclusion](docs/conclusion.md)
10. [References](docs/references.md)

## Quick Links
- [GitHub Repository](https://github.com/yourusername/yourrepository)
- [Issues](https://github.com/yourusername/yourrepository/issues)
- [Pull Requests](https://github.com/yourusername/yourrepository/pulls)

## Getting Started
To get started with the project, follow the [Setup Guide](docs/setup.md).

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments
- Thanks to all the contributors and maintainers of the ROS and Gazebo projects.
- Special thanks to the community for the support and resources.

