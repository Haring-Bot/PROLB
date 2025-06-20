from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    # Set the custom Nav2 parameters file path (without voxel grid)
    params_path = os.path.join(
        FindPackageShare('PROLB_Haring').find('PROLB_Haring'),
        'config', 'nav2_params_no_voxel.yaml'
    )

    # Gazebo Launch - using TurtleBot3 preinstalled worlds
    gazebo_pkg = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    gazebo_launch = os.path.join(gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')

    # Nav2 Launch - using TurtleBot3's navigation launch
    nav2_pkg = FindPackageShare('nav2_bringup').find('nav2_bringup')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')

    # Launch Gazebo simulation with TurtleBot3
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Launch Nav2 with our custom parameters file (without voxel grid)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_path,
            'map': map_file
        }.items()
    )

    # Map server node - load map specified by map_file argument
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_file}
        ]
    )

    # Lifecycle manager for Nav2
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 
                           'amcl',
                           'controller_server',
                           'planner_server',
                           'recoveries_server',
                           'bt_navigator']}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_file,
        gazebo,
        map_server,
        nav2,
        lifecycle_manager
    ])
