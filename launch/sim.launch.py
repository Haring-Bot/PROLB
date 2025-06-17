from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('PROLB_Haring')
    
    # Warehouse world is in turtlebot4_gz_bringup package
    turtlebot4_gz_bringup_pkg = get_package_share_directory('turtlebot4_gz_bringup')
    
    # Set the Gazebo resource path to include TurtleBot4 worlds
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(turtlebot4_gz_bringup_pkg, 'worlds'),  # TurtleBot4 worlds
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')  # Preserve existing path
        ])
    )
    
    # World file argument (default to 'warehouse')
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value='warehouse',
        description='World name or path to world file'
    )

    world_name = LaunchConfiguration('world')

    # For Gazebo Ignition/Garden (used in ROS 2 Jazzy)
    # The format is: gz sim -r <world_name>.sdf
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', [world_name, '.sdf']],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,  # Set this before launching Gazebo
        world_file_arg,
        gazebo
    ])
