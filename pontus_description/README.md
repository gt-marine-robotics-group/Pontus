# Pontus Description

## Contents
- [Launch Files](#launch-files)
  - [spawn.launch.py](#spawnlaunchpy)
- [Models](#models)
  - [Pontus](#pontus)
- [External Published Topics](#external-published-topics)

## Launch Files

### spawn.launch.py

Spawns the vehicle model in the currently running Gazebo simulation. Also runs the robot_state_publisher node to provide the TF tree and model description.

| Argument | Type | Description |
|----------|------|-------------|
| static | boolean | Setting this to true locks the vehicle in place in simulation |

## Models

### Pontus

The xacro vehicle description for the Robosub vehicle. Currently it is named Pontus but long term we will probably want a seperate name for the vehicle. The description is split into a few different files: 
- pontus.xacro: Contains the base link of the robot and uses the macros from other files to specify the locations of the thrusters, sensors, and actuators. Also configures the Gazebo plugin for hydrodynamics.
- components/thrusters.xacro: Thruster macros for the T200 and T500
- components/sensors.xacro: Macros for the various sensors on the vehicle (Camera, BlueView P900 Sonar, and IMU

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /TF | [tf2_msgs/TFMessage](https://docs.ros2.org/latest/api/tf2_msgs/msg/TFMessage.html) | N/A | TF tree of the model |
| /robot_description | [std_msgs/String](https://docs.ros2.org/latest/api/std_msgs/msg/String.html) | N/A | SDF description generated from the URDF |
