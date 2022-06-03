# ROS audio\_common Package
[![](https://github.com/swl017/audio_common/actions/workflows/main.yml/badge.svg?=master)](https://github.com/swl017/audio_common/actions/workflows/main.yml)

This repository is a fork from the [ros_drivers/audio_common](https://github.com/ros-drivers/audio_common) containing the ROS audio\_common package __and modifications for the AGC 2022 challenge.__


# Building from source

For the AGC 2022 competition, please `git clone` and build the `feature/wave` branch using `catkin_make`


# The concept and usage
이 패키지의 컨셉은 젯슨(`audio_pub.py`)에서 8 채널 오디오 데이터를 `/audio/all`/ 토픽으로 publish하고, 서버(`audio_sub.py`)에서 subscribe 받은 메세지를 각 채널로 분리하여 사용하는 것입니다. 따라서 아래와 같이 실행합니다. \
The concept is to publish 8 channel audio data to the topic `/audio/all` from Jetson(`audio_pub.py`), subscribe to the topic and split the audio channels on the server(`audio_sub.py`). Thus, the nodes are run as the following:

- Jetson side
```bash
rosrun audio_capture audio_pub.py
```

- Server side
```bash
rosrun audio_capture audio_sub.py
```

# Another possibility but not tested
[`capture_wave.launch`](https://github.com/swl017/audio_common/blob/feature/wave/audio_capture/launch/capture_wave.launch)


# Further Reference
The purpose of the provided code is to demonstrate an example to handle multi-channel audio data in ROS environment. You can to modify the provided code to fit your own needs.
