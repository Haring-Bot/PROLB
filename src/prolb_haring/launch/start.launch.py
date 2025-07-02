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
    #launch logger
    logger = get_logger("start.launch.py")
    
    #launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    autostart = LaunchConfiguration("autostart", default="true")
    use_respawn = LaunchConfiguration("use_respawn", default="False")
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true")
    
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", 
        default_value="true",
        description="Automatically startup the nav2 stack")
    
    #custom map
    pkg_share = FindPackageShare("prolb_haring").find("prolb_haring")
    map_path = os.path.join(pkg_share, "map", "map.yaml")
    map_file = LaunchConfiguration("map", default=map_path)

    logger.info("TESTTESTTEST")
    logger.info("!!!!!!! map path: %s" % map_path)

    #Gazebo Launch
    gazebo_pkg = FindPackageShare("turtlebot3_gazebo").find("turtlebot3_gazebo")
    gazebo_launch = os.path.join(gazebo_pkg, "launch", "turtlebot3_world.launch.py")

    #Nav2 Launch switched to standard nav to load custom configs
    nav2_pkg = FindPackageShare("nav2_bringup").find("nav2_bringup")
    nav2_launch = os.path.join(nav2_pkg, "launch", "bringup_launch.py")
    
    #TurtleBot3 parameters
    tb3_pkg = FindPackageShare("turtlebot3_navigation2").find("turtlebot3_navigation2")
    params_file = os.path.join(tb3_pkg, "param", "waffle.yaml")

    #map->odom transformation
    map_to_odom_static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_broadcaster",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )
    
    #initial pose for amcl
    initial_pose_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="initial_pose_pub",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "map", "base_footprint"]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )

    #custom RVIZ congig
    rviz_config_path = os.path.join(pkg_share, "config", "rviz_setup.rviz")
    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            "map": map_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": params_file,
            "use_rviz": "false"  #deactivate rviz launch to allow custom configs
        }.items()
    )

    #custom RVIZ
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    kf_node = Node(
        package="prolb_haring",
        executable="testnode",
        name="kalman_filter",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    nav_node = Node(
        package="prolb_haring",
        executable="navnode",
        name="navigation",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        gazebo,
        map_to_odom_static_transform,
        initial_pose_pub,
        nav2,
        rviz_node,
        kf_node,
        nav_node
    ])
