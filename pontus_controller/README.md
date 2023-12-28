# Pontus Controller

## Contents
- [Pontus Nodes](#pontus-nodes)
  - [velocity_controller.py](#velocity\_controllerpy)
  - [thruster_controller.py](#thruster\_controllerpy)
- [External Subscribed Topics](#external-subscribed-topics)
- [External Published Topics](#external-published-topics)
- [Parameters](#parameters)

## Pontus Nodes

### velocity_controller.py

Takes in a commanded body velocity on /cmd_vel and uses PID to generate body acceleration commands on /cmd_accel.

### thruster_controller.py

Takes in a commanded body acceleration on /cmd_accel and uses the thruster positions published on /tf to calculate individual thruster commands and publish them on /pontus/thruster_<id>/cmd_thrust.

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /cmd_vel | [std_msgs/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html) | base_link | Commanded body velocity of the vehicle, used by velocity_controller |
| /pontus/odometry | [nav_msgs/Odometry](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html) | odom | Used by velocity_controller |

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /pontus/thruster_\<id\>/cmd_thrust | [std_msgs/Float64](https://docs.ros2.org/latest/api/std_msgs/msg/Float64.html) | thruster_\<id\> | Commanded thrust in Newtons for specified thruster |

## Parameters

TODO: the controller nodes currently don't use external parameters but they should be updated to in the future. Eventually there will be parameters for the PID constants for the velocity controllers. Also it may be necessary to add parameters for the thruster_controller with information about the mass and inertia of the vehicle but it would be better to find a way to pull that information directly from the robot description
