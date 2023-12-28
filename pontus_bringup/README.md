# Pontus Bringup

## Contents
- [Launch Files](#launch-files)
  - [simulation.launch.py](#simulationlaunchpy)
  - [odom_simulation.launch.py](#odom\_simulationlaunchpy)

## Launch Files

### simulation.launch.py

This launch files starts the Gazebo simulation and spawns the vehicle in the world, then launches the robot_localization node and the velocity controller. NOTE: The auv argument is mandatory as some nodes will fail to launch without it.

| Argument | Type | Description |
|----------|------|-------------|
| auv | string | Which configuration folder to pull from. Current options are `robosub` and `sim_robosub` |
| static | boolean | Setting this to true locks the vehicle in place in simulation |
| world | file path | The world to launch with Gazebo |

### odom_simulation.launch.py

Launches all of the above nodes as well as a node to bridge the perfectly accurate vehicle odometry from Gazebo to ROS2 for testing. It might be better to make this an argument for the generic simulation.launch.py file.

| Argument | Type | Description |
|----------|------|-------------|
| auv | string | Which configuration folder to pull from. Current options are `robosub` and `sim_robosub` |
| static | boolean | Setting this to true locks the vehicle in place in simulation |
| world | file path | The world to launch with Gazebo |
