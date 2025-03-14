from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.actions import LogInfo
from launch.event_handlers import OnExecutionComplete
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
# from map_ready_check import MapReadyEvent  # Import custom event

def generate_launch_description():
    ld = LaunchDescription()

    # Odom TF Publisher Node
    odom_tf_publisher = Node(
        package='odom_tf_convert',
        executable='odom_tf_publisher',
        output='screen'
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
    )

    # LiDAR Launch
    lidar_launch_path = os.path.join(
        '/home/tori/ros2_ws/install/sllidar_ros2/share/sllidar_ros2/launch', 
        'view_sllidar_s1_launch.py'
    )
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path)
    )

    # Map Ready Check Node
    map_ready_check = Node(
        package='map_ready_check',
        executable='map_ready_check',
        output='screen'
    )

    # Localization Launch
    nav2_bringup_path = os.path.join(
        '/opt/ros/humble/share/nav2_bringup/launch', 'localization_launch.py'
    )
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_path),
        launch_arguments={
            'use_sim_time': 'false',
            'map': '/home/suki/ros2_ws/src/Unity-Coordinates/map_Unity1.yaml'
        }.items()
    )

    # Event to trigger localization after map is confirmed
    localization_trigger = RegisterEventHandler(
        OnExecutionComplete(
            target_action=map_ready_check,
            on_event=MapReadyEvent(),
            actions=[localization_launch]
        )
    )

    # Add nodes to launch description
    ld.add_action(odom_tf_publisher)
    ld.add_action(lidar_launch)
    ld.add_action(rviz_node)
    ld.add_action(map_ready_check)
    ld.add_action(localization_trigger)

    return ld

