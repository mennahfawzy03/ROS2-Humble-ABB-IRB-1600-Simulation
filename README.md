# ROS2 Enviornment setup 

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
