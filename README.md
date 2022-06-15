# ROS audio\_common Package
[![](https://github.com/swl017/audio_common/actions/workflows/main.yml/badge.svg?=master)](https://github.com/swl017/audio_common/actions/workflows/main.yml)

This repository is a fork from the [ros_drivers/audio_common](https://github.com/ros-drivers/audio_common) containing the ROS audio\_common package __and modifications for the AGC 2022 challenge.__


# Building from source

For the AGC 2022 competition, please `git clone` and build the `feature/wave` branch using `catkin_make`

```bash
cd catkin_ws/src
git clone https://github.com/swl017/audio_common -b feature/wave
cd ..
rosdep install --from-paths src/ --ignore-src -r -y
catkin_make
```

# Mic check
If `arecord -l` shows the `SpkUAC20` on `card 2`, the mic array is set and good to go.

(Example)
```bash
$ arecord -l
**** List of CAPTURE Hardware Devices ****
card 1: Generic [HD-Audio Generic], device 0: ALCS1200A Analog [ALCS1200A Analog]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 1: Generic [HD-Audio Generic], device 2: ALCS1200A Alt Analog [ALCS1200A Alt Analog]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 2: SpkUAC20 [miniDSP VocalFusion Spk (UAC2.0], device 0: USB Audio [USB Audio]
  Subdevices: 0/1
  Subdevice #0: subdevice #0
```

# The concept and usage
이 패키지의 컨셉은 젯슨(`audio_pub.py`)에서 8 채널 오디오 데이터를 `/audio/all`/ 토픽으로 publish하고, 서버(`audio_sub.py`)에서 subscribe 받은 메세지를 각 채널로 분리하여 사용하는 것입니다. 따라서 아래와 같이 실행합니다. \
The concept is to publish 8 channel audio data to the topic `/audio/all` from Jetson(`audio_pub.py`), subscribe to the topic and split the audio channels on the server(`audio_sub.py`). Thus, the nodes are run as the following:

## 1. Jetson side
```bash
rosrun audio_capture audio_pub.py
```
- This publishes audio data including all 8 channels in __`audio/all`__ topic. 
- The parameters are also published in __`audio/all/info`__ topic and are used in the subscriber node described below.
- For the moment, __only `pyhon2` is supported__ and __not `python3` due to differences in writing data into bytes__. Please push a PR if you are willing to share your solution.

## 2. Server side
```bash
rosrun audio_capture audio_sub.py
```
- `audio_sub.py` subscribes to __`audio/all` topic for the actual data__ and __`audio/all/info` topic for referencing the format of the data__.
- The subscriber node seperates the 8 channel data into individual channels. While the individual channels _are_ published in `audio/ch{CHANNEL}` topics, however, you may want to work with the seperated channels directly in the subscriber node to achieve maximum performance. (To get over with network delay, bandwidth, sync issues, etc.)
- You must select an appropriate length for each audio file in __`file_length_sec`__ variable.
- The subscriber node saves the original 8 channel audio into `wave_{NUMBER_CHANNELS}_{SAMPLE_FORMAT}_{SAMPLE_RATE}_{ROS_TIME_STAMP}.wav` and the individual channels in `wave_{NUMBER_CHANNELS}_{SAMPLE_FORMAT}_{SAMPLE_RATE}_{ROS_TIME_STAMP}_{CHANNEL}.wav`. The audio parameters and the names come from `audio/all/info` topic if it is ever subscribed, else default values are used. File saving is tested valid on `python2`. __Saving to `.wav` is not supported on `python3`__ for now. Please push a PR if you are willing to share your solution.

# Another possibility but not tested
[`capture_wave.launch`](https://github.com/swl017/audio_common/blob/feature/wave/audio_capture/launch/capture_wave.launch) from the original `audio_common` repository. Currently, it looks like mic arrays with channels more than 2 are not supported, though we need more investigation on this.


# Further Reference
The purpose of the provided code is to demonstrate an example to handle multi-channel audio data in ROS environment. You can to modify the provided code to fit your own needs.
