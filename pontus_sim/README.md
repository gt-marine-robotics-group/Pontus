# Pontus Sim

## Contents
- [Launch Files](#launch-files)
  - [sim.launch.py](#simlaunchpy)
  - [odom_bridge.launch.py](#odom\_bridgelaunchpy)
- [Worlds](#worlds)
  - [underwater.world](#underwaterworld)
  - [sensor.world](#sensorworld)
- [Models](#models)
  - [Gate](#gate)
- [External Subscribed Topics](#external-subscribed-topics)
- [External Published Topics](#external-published-topics)
- [Parameters](#parameters)
  - [bridge.yml](#bridgeyml)
  - [odom_bridge.yml](#odom\_bridgeyml)
- [Environment Hooks](#environment-hooks)
  - [pontus_sim.sh.in](pontus\_simshin)

## Launch Files

### sim.launch.py

Launches the Gazebo Simulation and a node that bridges internal Gazebo topics to ROS2. This allows us to access various simulated sensor and actuator topics from ROS2.

| Argument | Type | Description |
|----------|------|-------------|
| world | file path | The world to launch with Gazebo |

### odom_bridge.launch.py

This launches a node that specifically bridges the internal Gazebo odometry data for the vehicle over to ROS2. This should only be used for testing since the real vehicle will need to calculate this information from its sensors.

## Worlds

### underwater.world
The default world for simulation. Includes hydrodynamics and a blue tint to match the underwater lighting as well as lane markings from a pool, and various robosub tasks.

### sensor.world
A world specifically for testing sensor data on the vehicle. The vehicle is held statically in position and there are no hydrodynamics so it is easier to place objects for the sensors to look at without having to worry about them floating away. Also includes the lidar visulization panel on the left for displaying the sonar beams.

## Models

### Gate
The gate used for the [Robosub Gate Task](https://robonation.org/app/uploads/sites/4/2023/06/2023-RoboSub_Team-Handbook_v2.0.pdf#page=17)

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /cmd_vel | [std_msgs/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html) | base_link | Commanded body velocity of the vehicle, used by velocity_controller |
| /pontus/odometry | [nav_msgs/Odometry](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html) | odom | Used by velocity_controller |

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /cmd_accel | [std_msgs/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html) | base_link | Commanded body acceleration of the vehicle |
| /pontus/thruster_\<id\>/cmd_thrust | [std_msgs/Float64](https://docs.ros2.org/latest/api/std_msgs/msg/Float64.html) | thruster_\<id\> | Commanded thrust in Newtons for specified thruster |

## Parameters

### bridge.yml
Contains the mappings from Gazebo topics to ROS2 topics. Specifically this includes a section for all of the vehicles sensors, a section for thrusters, and a section for the Gazebo simulation clock.

### odom_bridge.yml
Contains the mapping for the perfectly accurate simulation odometry. This should only be used for testing since the real vehicle will need to calculate this information from its sensors.

## Environment Hooks

### pontus_sim.sh.in
This file is used to add the model and world folders to the `$GZ_SIM_RESOURCE_PATH` so that Gazebo can use the files there. There is a bit of a mix up with GZ and IGN in some of the environment variables for Gazebo Garden but hopefully this will be fixed in the next release.
