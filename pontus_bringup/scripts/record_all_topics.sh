#!/bin/bash
ros2 bag record -a --exclude-topic-types sensor_msgs/msg/Image "$@"
