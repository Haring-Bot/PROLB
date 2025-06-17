from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_tb4 = get_package_share_directory('turtlebot4_gz_bringup')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4, 'launch', 'sim.launch.py'])
        )
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4, 'launch', 'turtlebot4_spawn.launch.py'])
        ),
        launch_arguments={
            'x': '0.0',
            'y': '0.0',
            'z': '0.01',
            'yaw': '0.0',
            'model': 'standard',
            'rviz': 'false'
        }.items()
    )

    return LaunchDescription([gazebo, spawn])
