import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node 
import xacro


def generate_launch_description():

	robotXacroName='simple_robot'
	
	
	
	
	namePackage = 'gazebo_test'
	
	
	modelFileRelativePath = 'model/robot.xacro'
	
	worldFileRelativePath = 'model/robot.xacro'
	
	
	pathModelFile = os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)
	
	
	pathWorldFile = os.path.join(get_package_share_directory(namePackage),worldFileRelativePath)
	
	robotDescription = xacro.process_file(pathModelFile).toxml()
	
	
	
	gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'))
	
	
	
	gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch,launch_arguments={'world':pathWorldFile.items())
	
	
	spawnModelNode = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic','robot_description','-entity',robotXacroName], output='screen')
	
	
	
	nodeRobotStatePublisher = Node(
		package='robot_state_publisher'
		executable  
