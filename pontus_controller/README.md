# Pontus Controller

## Contents
- [Controlling the Vehicle](#controlling-the-vehicle)
  - [Command Mode](#command-mode)
  - [Usage for Manual Mode](#usage-for-manual-control)
  - [Usage in Pontus Autonomy](#usage-in-pontus-autonomy)
- [Pontus Nodes](#pontus-nodes)
  - [position_controller.py](#position\_controllerpy)
  - [velocity_controller.py](#velocity\_controllerpy)
  - [thruster_controller.py](#thruster\_controllerpy)
- [External Subscribed Topics](#external-subscribed-topics)
- [External Published Topics](#external-published-topics)
- [Parameters](#parameters)
  - [Position Controller](#position-controller)
  - [Velocity Controller](#velocity-controller)

## Controlling the Vehicle
### Command Mode
The controller system supports several different modes which can be set by publishing a `CommandMode` message to the `/command_mode` topic or modifying the `command_mode` field in a `GoToPose` action to one of the following numbers:

0. **E-Stop**: Software level E-Stop that zeroes out the thruster then stops sending commands.
1. **Direct Control**: The `/cmd_vel` topic is passed directly into the thrusters
2. **Velocity Control**: Feedback from the odometry is used to match the `/cmd_vel` topic.
3. **Velocity Hold Depth**: Same as **Velocity Control** but when 0 velocity is commanded for the z-axis the position controller maintains the depth.
4. **Velocity Hold Heading**: Same as **Velocity Hold Depth** but also holds the heading.
5. **Velocity Hold Position**: Same as **Velocity Hold Depth** but holds every degree of freedom.
6. **Position Face Travel**: Standard position controller mode, uses feedback from the odometry to move to the commanded position on the `/cmd_pos` topic. First moves to the commanded depth, points in the direction of the target point, then moves to it. Once it reaches the target point it turns to face the commanded orientation.
7. **Position With Strafe**: Uses feedback from the odometry to move directly to the commanded position and orientation.

### Usage for manual control
1. Start the hardware and controller nodes by running the `auv.launch.py` file on the vehicle.
2. Controlling the vehicle:
    * Using the joystick controller:
        1. Launch the `rc.launch.py` launch file on the laptop
        2. Set the command mode using the buttons:
            * **E-Stop** B (red button)
            * **Direct Control** X (blue button)
            * **Velocity Control** Y (yellow button)
            * **Velocity Hold Position** A (green button)
            * **Position Face Travel** Start button
        3. Drive the vehicle using the joystick. (In **Position Face Travel** mode it is expected that autonomy code will be used to control the vehicle)
    * Using RQT or Foxglove:
        1. Set the command mode by publishing to the `/command_mode` topic.
        2. Control the vehicle by publishing to `/cmd_vel` or `/cmd_pos`
            * When using the **Direct Control** or **Velocity Control** modes only the `/cmd_vel` is used.
            * When using the Velocity with hold modes the `/cmd_vel` is primarily used but a new position to be held can be published to the `/cmd_pos` topic.
            * When using the **Position Face Travel** or **Position With Strafe** modes only the `/cmd_pos` is used.

### Usage in Pontus Autonomy
The `pontus_autonomy` package provides a helper class for controlling the vehicle called `GoToPoseClient` which simplifies the process of sending `GoToPose` actions. To use the `GoToPoseClient`:
1. Initialize the `GoToPoseClient` object in the constructor of your node: `self.go_to_pose_client = GoToPoseClient(self)`
2. Construct a new `PoseObj` object representing the intended command: `pose_obj = PoseObj(cmd_pose, cmd_twist, skip_orientation, use_relative_position, command_mode)`
    * `cmd_pose`: A ROS Pose message containing the intended position and orientation. Defaults to `None` if not provided
    * `cmd_twist`: A ROS Twist message containing the intended linear and angular velocities. Defaults to `None` if not provided
    * `skip_orientation`: A boolean indicating if the controller should actually turn to the final orientation commanded in the `cmd_pose` which is useful for intermediate points on a path where the vehicle should immediately begin moving to the next point. Defaults to `False` if not provided.
    * `use_relative_position`: A boolean indicating if the controller should interpret the `cmd_pos` as being in the map frame or relative to the vehicles current position. Defaults to `False`
    * `command_mode`: An integer indicating the command mode for the controller to use. Defaults to `CommandMode.POSITION_FACE_TRAVEL` which is the most commonly used mode for autonomy.
3. Send the command: `self.go_to_pose_client.go_to_pose(pose_obj)`

## Pontus Nodes

### position_controller.py

Takes in a commanded mode, a commanded position on `/cmd_pos`, and a commanded velocity on `/cmd_vel` then depending on the mode forwards the commanded velocity or uses PID to generate body velocity commands on `/cmd_vel_fused`.

### velocity_controller.py

Takes in a commanded body velocity on `/cmd_vel_fused` and uses PID to generate body acceleration commands on `/cmd_accel`.

### thruster_controller.py

Takes in a commanded body acceleration on `/cmd_accel` and uses the thruster positions published on `/tf` to calculate individual thruster commands and publish them as an array of thruster efforts between -1 and 1 on `/thrust_cmds`.

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /command_mode | pontus_msgs/CommandMode | N/A | Specific mode for the position and velocity controllers to run in |
| /cmd_pos | [std_msgs/Pose](https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html) | map | Commanded position of the vehicle, used by position_controller |
| /cmd_vel | [std_msgs/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html) | base_link | Commanded body velocity of the vehicle, used by velocity_controller |
| /pontus/odometry | [nav_msgs/Odometry](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html) | odom | Used by velocity_controller and position_controller |

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /thruster_cmds | [std_msgs/Float32MultiArray](https://docs.ros2.org/latest/api/std_msgs/msg/Float32MultiArray.html) | N/A | Commanded thruster effort between -1 and 1 |

## Parameters

### Position Controller
* `lookahead_distance`: The lookahead distance used when following the line drawn between the starting point of the vehicle and its commanded position
* Velocity Limits: Specific limits in m/s or radian/s for certain degrees of freedom:
    * `x_vmax`
    * `y_vmax`
    * `yaw_vmax`
* Position PID Gains: P, I, and D Gains for each degree of freedom
    * `x_kp`
    * `x_ki`
    * `x_kd`
    * `y_kp`
    * `y_ki`
    * `y_kd`
    * `z_kp`
    * `z_ki`
    * `z_kd`
    * `r_kp`
    * `r_ki`
    * `r_kd`
    * `p_kp`
    * `p_ki`
    * `p_kd`
    * `yaw_kp`
    * `yaw_ki`
    * `yaw_kd`

### Velocity Controller
* Direct Mode Gains: Multipliers for linear or angular commands when using **Direct Control** mode
    * `direct_mode_linear_gain`
    * `direct_mode_angular_gain`
* Coefficient of drags for each degree of freedom:
    * `x_C`
    * `y_C`
    * `z_C`
    * `r_C`
    * `p_C`
    * `yaw_C`
* Velocity PID Gains: P, I, and D Gains for each degree of freedom
    * `x_kp`
    * `x_ki`
    * `x_kd`
    * `y_kp`
    * `y_ki`
    * `y_kd`
    * `z_kp`
    * `z_ki`
    * `z_kd`
    * `r_kp`
    * `r_ki`
    * `r_kd`
    * `p_kp`
    * `p_ki`
    * `p_kd`
    * `yaw_kp`
    * `yaw_ki`
    * `yaw_kd`
