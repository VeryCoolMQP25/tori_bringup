from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # Odom TF Publisher Node
    odom_tf_publisher = Node(
        package='odom_tf_convert',
        executable='odom_tf_publisher',
        output='screen'
    )

    # # Goal Pose Filter Node
    goal_pose_filter = Node(
        package='goal_pose_filter', 
        executable='goal_pose_filter',
        output='screen'
    )

    
    # # Check goal dist
    check_goal_dist = Node(
        package='check_goal_proximity', 
        executable='check_goal_proximity',
        output='screen'
    )

    # Light commander
    light_commander = Node(
        package='LightCommander', 
        executable='light_commander',
        output='screen'
    )

    # Elevator State manager
    transition_manager = Node(
        package='elevator_transitions',
        executable='transitions',
        output='screen'
    )

    elevator_orchestrator = Node(
        package='elevator_orchestrator',
        executable='orchestrator',
        output='screen'
    )
    
    vision = Node(
        package='vision',
        executable='button_coordinate_publisher',
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
        '/home/tori/ros2_ws/install/sllidar_ros2/share/sllidar_ros2/launch', 'sllidar_s1_launch.py' # jake change path here
    )
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        launch_arguments={
            'serial_port': '/dev/LiDAR'
        }.items()
    )
    rosbridge_launch_file = os.path.join(
        get_package_share_directory("rosbridge_server"),
        "launch",
        "rosbridge_websocket_launch.xml"
    )
    rosbridge_launch =  IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch_file)
        )

    # # Map Loader Node
    map_loader_node = Node(
        package='map_loader',
        executable='map_loader',
        output='screen'
    )
    # battery_node = Node(
            # package='battery_monitor',
            # executable='battery_monitor',
            # output='screen'
        # )
    # Localization Launch
    nav2_loc_bringup_path = os.path.join('/opt/ros/humble/share/nav2_bringup/launch', 'localization_launch.py')
    
    localization_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_loc_bringup_path),
                    launch_arguments={
                        'map': '/home/tori/Maps/map_Unity1.yaml'  # change path here too
                    }.items()
                    )

    nav2_nav_bringup_path = os.path.join('/opt/ros/humble/share/nav2_bringup/launch', 'navigation_launch.py')
    
    navigation_launch = TimerAction(
        period=4.0,  # wait for localization to come up
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_nav_bringup_path)
            )
        ]
    )
    # Add nodes to launch description
    ld.add_action(odom_tf_publisher)
    ld.add_action(goal_pose_filter)
    ld.add_action(check_goal_dist)
    ld.add_action(light_commander)
    ld.add_action(lidar_launch)
    # ld.add_action(battery_node)
    ld.add_action(map_loader_node)
    ld.add_action(localization_launch)
    ld.add_action(navigation_launch)
    ld.add_action(transition_manager)
    # ld.add_action(elevator_orchestrator)
    # ld.add_action(vision)

    return ld
