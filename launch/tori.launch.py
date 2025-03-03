from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    ld = LaunchDescription()

    # Odom TF Publisher Node
    odom_tf_publisher = Node(
        package='odom_tf_convert',
        executable='odom_tf_publisher',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
    )

    lidar_launch_path = os.path.join("/home/tori/ros2_ws",
        'install/sllidar_ros2/share/sllidar_ros2/launch', 'view_sllidar_s1_launch.py'
    )
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path)
    )
    collision_launch_path = os.path.join("/opt/ros/humble/share/nav2_collision_monitor/launch/collision_monitor_node.launch.py")
    collision_monitor_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(collision_launch_path))
    nav2_bringup_path = os.path.join(
        '/opt/ros/humble/share/nav2_bringup/launch', 'localization_launch.py'
    )
    localization_launch = TimerAction(
        period=3.0,  # localization node launched after 3 sec delay to ensure map server has enough time to load map into Rviz
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup_path),
                launch_arguments={
                    'use_sim_time': 'false',
                    'map': "/home/tori/Maps/floor4.yaml"
                }.items()
            )
        ]
    )

    ld.add_action(odom_tf_publisher)
    ld.add_action(lidar_launch)
    ld.add_action(rviz_node)
    ld.add_action(localization_launch)
    ld.add_action(collision_monitor_launch) 

    return ld
