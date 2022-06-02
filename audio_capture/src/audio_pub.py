#!/usr/bin/env python
'''
@file   audio_pub.py
@brief  Publish audio messages in AudioDataStringArray.
@date   2022-06-01
@author Seungwook Lee, USRG@KAIST
@todo   Investigate device option like arecord -l
@todo   Investigate how to specify the bitrate
'''

import rospy
import pyaudio

from audio_common_msgs.msg import AudioDataStringArray
from audio_common_msgs.msg import AudioInfo

class WavAudioPub:
    def __init__(self, pub_hz):
        self.chunk          = 1024  # Record in chunks of 1024 samples
        self.sample_format  = pyaudio.paInt16  # 16, 32 supported
        self.channels       = 8
        self.fs             = 44100  # Record at 44100 samples per second
        self.seconds        = 1.0 / pub_hz
        self.p = pyaudio.PyAudio()  # Create an interface to PortAudio
        self.stream = self.p.open(format=self.sample_format,
                                channels=self.channels,
                                rate=self.fs,
                                frames_per_buffer=self.chunk,
                                input=True)

        self.frame_pub = rospy.Publisher("audio/all", AudioDataStringArray, queue_size=1)
        self.info_pub  = rospy.Publisher("audio/all/info", AudioInfo, queue_size=1)

        print('Node initiated: audio_pub')
        
    def getAudioFrames(self):

        frames = []  # Initialize array to store frames

        # Store data in chunks for x seconds
        for i in range(0, int(self.fs / self.chunk * self.seconds)):
            data = self.stream.read(self.chunk)
            frames.append(data)

        self.publish(frames)

    def publish(self, frames):
        msg = AudioDataStringArray()
        msg.header.stamp = rospy.Time.now()
        msg.data = frames
        self.frame_pub.publish(msg)

        info = AudioInfo()
        info.channels = self.channels
        info.sample_rate = self.fs
        info.sample_format = str(self.sample_format)
        self.info_pub.publish(info)
        
    def terminateStream(self):
        # Stop and close the stream 
        self.stream.stop_stream()
        self.stream.close()
        # Terminate the PortAudio interface
        self.p.terminate()
        print('Node terminated')


def main():
    rospy.init_node('audio_pub')

    pub_hz = 10
    rate   = rospy.Rate(pub_hz) # 10hz

    node = WavAudioPub(pub_hz)
    while not rospy.is_shutdown():
        node.getAudioFrames()
        rate.sleep()
    
    node.terminateStream()

if __name__=="__main__":
    main()