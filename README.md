# Tori Localization Setup

## Overview

This README provides the steps to set up and run localization for the Tori robot.

Running the tori.launch.py file should replace the following commands:

``` ros2 launch sllidar_ros2 view_sllidar_s1_launch.py```

``` ros2 run odom_tf_convert odom_tf_publisher```

``` ros2 launch nav2_bringup localization_launch.py```

## Localization Setup for Tori

To run localization (using a personal device), use the following commands:

``` ros2 launch tori_bringup tori.launch.py ```

```ros2 run tf2_ros static_transform_publisher .14 0 1.12 0 0 0  base_link laser```

```agent```

``` rviz2```

To run localization on the Jetson, use the following commands:

``` ros2 launch tori_bringup tori.launch.py ```

``` rviz2```

## Navigation Setup for Tori

To run navigation, use the following command along with the ones under localization:

``` ros2 launch nav2_bringup navigation_launch.py```
