# ROS audio\_common Package
[![](https://github.com/ros-drivers/audio_common/actions/workflows/main.yml/badge.svg?=master)](https://github.com/ros-drivers/audio_common/actions/workflows/main.yml)

This repository is a fork from the [ros_drivers/audio_common](https://github.com/ros-drivers/audio_common) containing the ROS audio\_common package __and modifications for the AGC 2022 challenge.__


# Building from source

For the AGC 2022 competition, please `git clone` and build the `feature/wave` branch using `catkin_make`


# The concept and usage
The concept is to publish 8 channel audio data to the topic `/audio/all` from Jetson(`audio_pub.py`), subscribe to the topic and split the audio channels on the server(`audio_sub.py`). Thus, the nodes are run as the following:

- Jetson side
```bash
rosrun audio_capture audio_pub.py
```

- Server side
```bash
rosrun audio_capture audio_sub.py
```


# Further Reference
The purpose of the provided code is to demonstrate how we can handle multi-channel audio data in ROS environment. It is strongly recommended to optimize the provided code to fit your own needs.
