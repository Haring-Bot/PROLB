from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from launch.logging import get_logger
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Get a logger for this launch file
    logger = get_logger('full_bringup.launch.py')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    use_respawn = LaunchConfiguration('use_respawn', default='False')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    # Update the default map path to point to your package's map
    pkg_share = FindPackageShare('turtlebot3_full_bringup').find('turtlebot3_full_bringup')
    map_path = os.path.join(pkg_share, 'map', 'map.yaml')
    map_file = LaunchConfiguration('map', default=map_path)

    # Use logger calls for debug output
    logger.info("TESTTESTTEST")
    logger.info("!!!!!!! map path: %s" % map_path)

    # Gazebo Launch
    gazebo_pkg = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    gazebo_launch = os.path.join(gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')

    # Nav2 Launch
    nav2_pkg = FindPackageShare('turtlebot3_navigation2').find('turtlebot3_navigation2')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation2.launch.py')

    # Create static transform publishers for the missing transforms
    # This provides the map -> odom transform that AMCL would typically provide
    map_to_odom_static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # This node publishes the initial pose to initialize AMCL
    initial_pose_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='initial_pose_pub',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint']
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
            'autostart': autostart
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        gazebo,
        # Add the static transform publishers before nav2
        map_to_odom_static_transform,
        initial_pose_pub,
        nav2
    ])
