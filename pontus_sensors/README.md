# Pontus Sensors

## Contents
- [Pontus Nodes](#pontus-nodes)
  - [ros_blueview_driver](#ros\_blueview\_driver)
- [External Published Topics](#external-published-topics)
- [Parameters](#parameters)
  - [blueview.yaml](blueviewyaml)

## Pontus Nodes

### ros_blueview_driver
ROS2 node for interacting with the BlueView P900 sonar. If you intend to use this node make sure to run `git submodule update --init --recursive` inside the git repo to get the BlueView SDK from our private repo.

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /image_mono | [sensor_msgs/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html) | sonar_0 | Mono16 encoded sonar image in R-Theta format |
| /image_color | [sensor_msgs/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html) | sonar_0 | BRGA8 encoded sonar image in R-Theta format |

## Parameters

### blueview.yaml
| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| ros_blueview_driver | sonar_frame | TF frame | Name of the sonar TF frame. |
| ros_blueview_driver | grayscale | boolean | Set this to true to monochannel sonar images. |
| ros_blueview_driver | color | boolean | Set this to true to publish colorized sonar images. Only really useful for humans to view |
| ros_blueview_driver | raw | boolean | Set this to true to publish raw sonar data. Unfortunately the P900 does not support this so leave this false |
| ros_blueview_driver | range/start | double | Lowest distance to consider valid, closer points are discarded |
| ros_blueview_driver | range/stop | double | Highest distance to consider valid, farther points are discarded |
| ros_blueview_driver | fluid_type | string | Type of fluid the sonar is in, options are `freshwater`, `saltwater`, or `other` |
| ros_blueview_driver | sound_speed | int | Speed of sound in the fluid in meters per second |
| ros_blueview_driver | range_profile_intensity_threshold | int | Lowest intensity added to range messages, lower values are discarded  |
| ros_blueview_driver | noise_threshold | double | Threshold of noise to filter between 0.0 and 1.0  |
| ros_blueview_driver | period_seconds | double | How often to send an output, negative values will return as soon as an image is available |
| ros_blueview_driver | device | string | the IP address to communicate with the sonar. Leave this unset if you intend to read from a file instead |
| ros_blueview_driver | file | string | the filename to read from, leave this unset if you intend to use the actual sonar |
| ros_blueview_driver | color/map_file | string | the filename of the color mapping file, only necessary if color is enabled |
