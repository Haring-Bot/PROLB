from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map', default=os.path.expanduser('~/map.yaml'))

    # Gazebo Launch
    gazebo_pkg = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    gazebo_launch = os.path.join(gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')

    # Nav2 Launch
    nav2_pkg = FindPackageShare('turtlebot3_navigation2').find('turtlebot3_navigation2')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation2.launch.py')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([gazebo, nav2])
