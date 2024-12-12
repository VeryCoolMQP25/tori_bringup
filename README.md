# Tori Localization Setup

## Overview

This README provides the steps to set up and run localization for the Tori robot.

Running this launch file should replace the following commands:

``` ros2 launch sllidar_ros2 view_sllidar_s1_launch.py```

``` ros2 run odom_tf_convert odom_tf_publisher```

``` rviz2```

``` ros2 launch nav2_bringup localization_launch.py```

## Localization Setup for Tori

To run localization, use the following command:

```
ros2 launch tori_bringup tori.launch.py 
