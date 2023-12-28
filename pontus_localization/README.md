# Pontus Localization

## Contents
- [External Subscribed Topics](#external-subscribed-topics)
- [External Published Topics](#external-published-topics)
- [Parameters](#parameters)

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /pontus/imu_0 | [sensor_msgs/IMU](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html) | imu_0 | Used by EKF |

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /pontus/odometry | [nav_msgs/Odometry](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html) | odom | Used for navigation. |

## Parameters

### ekf.yaml
Documentation for the parameters can be found [here](http://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html).
