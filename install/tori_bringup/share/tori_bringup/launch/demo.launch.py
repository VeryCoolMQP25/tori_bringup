from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    ld = LaunchDescription()

    odom_tf_publisher = Node(
        package='odom_tf_convert',
        executable='odom_tf_publisher',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    nav2_bringup_path = os.path.join(
        '/opt/ros/humble/share/nav2_bringup/launch', 'localization_launch.py'
    )
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_path),
        launch_arguments={
            'use_sim_time': 'false',
            'map': '/home/suki/map_Dec7-1.yaml'
        }.items()
    )

    ld.add_action(odom_tf_publisher)
    ld.add_action(rviz_node)
    ld.add_action(localization_launch)

    return ld
