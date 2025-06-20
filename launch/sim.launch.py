from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Create LaunchConfiguration objects for the arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Set the map file path here:
    default_map = os.path.join(
        FindPackageShare('PROLB_Haring').find('PROLB_Haring'),
        'map', 'map.yaml'
    )
    
    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load'
    )
    map_file = LaunchConfiguration('map')

    # Gazebo Launch
    gazebo_pkg = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    gazebo_launch = os.path.join(gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')

    # Nav2 Launch with custom params
    nav2_pkg = FindPackageShare('turtlebot3_navigation2').find('turtlebot3_navigation2')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation2.launch.py')
    
    # Path to your custom nav2 params
    params_file = os.path.join(
        FindPackageShare('PROLB_Haring').find('PROLB_Haring'),
        'config', 'nav2_params.yaml'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file  # Add this line to use your custom params
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_file,
        gazebo,
        nav2
    ])
