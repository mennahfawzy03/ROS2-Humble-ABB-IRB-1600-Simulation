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
