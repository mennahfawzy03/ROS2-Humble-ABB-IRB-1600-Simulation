# ROS 2 Humble General Information and Best Practices

This document provides detailed explanations and instructions on how to work with ROS 2 Humble, including creating and navigating directories, understanding workspaces, creating packages, cloning repositories, and more.

---

## 1. Creating and Navigating Directories in ROS 2

### Creating Directories
To create directories in Linux (and thus in ROS), use the `mkdir` command:
```bash
mkdir -p ~/your_workspace_name/src
```
*This command creates a new directory named `your_workspace_name` with a subdirectory `src`. The `-p` flag ensures that parent directories are created as needed.*

### Navigating Directories
To navigate directories, use the `cd` command:
```bash
cd ~/your_workspace_name/src
```
*This changes the current working directory to `src` within your workspace.*

---

## 2. Understanding Workspaces

### What is a Workspace?
A workspace in ROS 2 is a directory where your ROS 2 packages are built, stored, and sourced. It typically contains a `src` folder where the source code resides, a `build` directory for compiled files, an `install` directory where the built files are stored, and a `log` directory for logs.

### Creating a Workspace
To create a workspace, use the following commands:
```bash
mkdir -p ~/ws_name/src
cd ~/ws_name
colcon build
```
*The `colcon build` command is used to compile the packages within the workspace.*

### Sourcing the Workspace
To use the packages in your workspace, source it:
```bash
source ~/ws_name/install/setup.bash
```
*Sourcing the setup file allows your current shell session to recognize the packages and environment variables in your workspace.*

### Automatically Sourcing the Workspace
To automatically source your workspace every time you open a terminal, you can add the sourcing command to your `.bashrc` file. Here’s how:

1. Open the `.bashrc` file in a text editor:
   ```bash
   gedit ~/.bashrc
   ```
   *This command opens the `.bashrc` file in the Gedit text editor. You can replace `gedit` with another editor like `nano` or `vim` if you prefer.*

2. Scroll to the bottom of the file and add the following line:
   ```bash
   source ~/ws_name/install/setup.bash
   ```
   *This line will be executed every time you open a new terminal, automatically sourcing your workspace.*

3. Save the file and close the editor.

4. To apply the changes immediately, run:
   ```bash
   source ~/.bashrc
   ```

*This command reloads the `.bashrc` file in your current session, applying the changes without needing to open a new terminal.*

---

## 3. Creating ROS 2 Packages

### What is a Package?
A ROS 2 package is the basic unit of software in ROS. It contains everything needed to build and run the software, including code, libraries, configuration files, and more.

### Creating a New Package
To create a new package, use the following command:
```bash
ros2 pkg create --build-type ament_cmake my_package_name
```
*This creates a new package named `my_package_name` with `ament_cmake` as the build system.*

### Package Structure
A typical ROS 2 package structure includes:
- **CMakeLists.txt**: Defines how the package is built.
- **package.xml**: Contains metadata about the package, such as its name, version, and dependencies.
- **src/**: Contains the source code.
- **include/**: Contains header files (if applicable).
- **launch/**: Contains launch files, which are used to start nodes and set up the environment.

### Explanation of Key Files:
- **CMakeLists.txt**: Defines the build process, linking libraries, and setting up executables.
- **package.xml**: Lists package dependencies, build dependencies, and version information.
- **launch files**: Used to launch multiple nodes, set parameters, and manage the runtime environment.

---

## 4. Cloning Repositories from GitHub

### Cloning a Repository
To clone a repository from GitHub, use:
```bash
git clone https://github.com/username/repository_name.git
```
*This command clones the specified GitHub repository into your current directory.*

### Adding Cloned Packages to Your Workspace
After cloning, move the package into your workspace’s `src` directory:
```bash
mv repository_name ~/ws_name/src/
cd ~/ws_name
colcon build
```
*Sourcing the workspace after building allows you to use the cloned package.*

---

## 5. Types of Files in ROS 2 and How to Create Them

### Launch Files
Launch files are used to start nodes and set up parameters. Create a launch file:
```bash
touch ~/ws_name/src/my_package/launch/my_launch_file.launch.py
```
*This creates an empty launch file where you can define how your nodes are launched.*

### Node Files
Nodes are executable processes that perform computations. Create a node file:
```bash
touch ~/ws_name/src/my_package/src/my_node.cpp
```
*This creates an empty C++ source file where you can write your node’s code.*

### URDF Files
URDF (Unified Robot Description Format) files describe the physical configuration of a robot. Create a URDF file:
```bash
touch ~/ws_name/src/my_package/urdf/my_robot.urdf
```
*This creates an empty URDF file where you can define your robot model.*

### Configuration Files
Configuration files store parameters that nodes use. Create a YAML configuration file:
```bash
touch ~/ws_name/src/my_package/config/my_config.yaml
```
*This creates an empty YAML file for storing configuration parameters.*
