import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import os

def generate_launch_description():
    pkgPath = launch_ros.substitutions.FindPackageShare(package='manipulator').find('manipulator')
    urdfModelPath= os.path.join(pkgPath, 'urdf/models.urdf')
    rvizConfigPath= os.path.join(pkgPath, 'config/config.rviz')
    
    print(urdfModelPath)
    with open(urdfModelPath, 'r') as infp:
        robot_desc = infp.read()
        
    params = {'robot_description': robot_desc}
    
    robot_state_publisher_node =launch_ros.actions.Node(
    package='robot_state_publisher', 
    executable='robot_state_publisher',
    output='screen',
    parameters=[params],
    arguments=[urdfModelPath])
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        arguments=[urdfModelPath]
    )   
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[params],
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui')) 
    )   
    rviz_node = launch_ros.actions.Node(
        package = 'rviz2',
        executable='rviz2',
        name ='rviz2',
        output='screen',
        arguments=['-d',rvizConfigPath]
    )   
    
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui',default_value='True',
                                            description='This is a flag for joint_state_publisher_gui'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ]) 


