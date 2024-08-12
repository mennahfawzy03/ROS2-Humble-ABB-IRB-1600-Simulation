## ROS 2 Humble Installation on Ubuntu 22.04

To set up ROS 2 Humble Hawksbill on Ubuntu for development, you'll need to follow these steps:

1. **Set Locale:**
   ```bash
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   ```

2. **Add the ROS 2 apt repository:**
   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

3. **Install Development Tools and ROS Tools:**
   ```bash
   sudo apt update && sudo apt install -y python3-flake8-docstrings python3-pip python3-pytest-cov ros-dev-tools
   sudo apt install -y python3-flake8-blind-except python3-flake8-builtins python3-flake8-class-newline python3-flake8-comprehensions python3-flake8-deprecated python3-flake8-import-order python3-flake8-quotes python3-pytest-repeat python3-pytest-rerunfailures
   python3 -m pip install -U flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-import-order flake8-quotes "pytest>=5.3" pytest-repeat pytest-rerunfailures
   ```

4. **Get ROS 2 Code:**
   ```bash
   mkdir -p ~/ros2_humble/src
   cd ~/ros2_humble
   vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src
   ```

5. **Install Dependencies:**
   ```bash
   sudo apt upgrade
   sudo rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
   ```

6. **Build the Workspace:**
   ```bash
   cd ~/ros2_humble/
   colcon build --symlink-install
   ```

7. **Environment Setup:**
   ```bash
   . ~/ros2_humble/install/local_setup.bash
   ```

8. **Try an Example:**
   In one terminal:
   ```bash
   . ~/ros2_humble/install/local_setup.bash
   ros2 run demo_nodes_cpp talker
   ```
   In another terminal:
   ```bash
   . ~/ros2_humble/install/local_setup.bash
   ros2 run demo_nodes_py listener
   ```

For further details, follow the instructions from the [ROS 2 Humble Ubuntu Development Setup Guide](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

## Blender installation
Blender is a powerful open-source software used for 3D creation. It is capable of modeling, sculpting, animation, rendering, and more. In this setup, Blender will be used to visualize STL files, scale down large STL meshes, ensure they are correctly scaled, and verify the position of the meshes to know what translation is needed. 
Blender was installed from [blender.org](https://www.blender.org/download/)
After downloading Blender, unzip the downloaded file and extract all files into your home directory (or any other preferred directory). Open the application by launching the blender application from your Files App. 

![image](https://github.com/user-attachments/assets/801d5318-43b7-4238-838f-78603bf0d1dc)


## Gazebo Installation 
Gazebo is already included within the ROS2 Package installed earlier. However, to be able to run gazebo and open it you'll have to install the command first using the following:
```bash
sudo apt install gazebo
```
Now, you can open gazebo by simply running `gazebo` in your terminal. 

## Visual Studio Code 
(you dont need it you can use simple text editor)


