#!/usr/bin/env python
'''
@file   pyaudio_pub8.py
@brief  Publish audio messages in AudioDataStringArray.
@date   2022-06-01
@author Seungwook Lee, USRG@KAIST
'''

import rospy
import pyaudio

from audio_common_msgs.msg import AudioData
from audio_common_msgs.msg import AudioDataStringArray

class RosPyaudio:
    def __init__(self, pub_hz):
        self.chunk          = 1  # Record in chunks of 1024 samples
        self.sample_format  = pyaudio.paInt16  # 16 bits per sample
        self.channels       = 8
        self.fs             = 44100  # Record at 44100 samples per second
        self.seconds        = 1.0 / pub_hz
        self.p = pyaudio.PyAudio()  # Create an interface to PortAudio
        self.stream = self.p.open(format=self.sample_format,
                                channels=self.channels,
                                rate=self.fs,
                                frames_per_buffer=self.chunk,
                                input=True)

        self.frame_pub = rospy.Publisher("audio/all_channels", AudioDataStringArray, queue_size=1)
        
    def setHz(self, hz):
        self.seconds = 1.0 / hz

    def getAudioFrames(self):

        frames = []  # Initialize array to store frames

        # Store data in chunks for x seconds
        print_once = True
        for i in range(0, int(self.fs / self.chunk * self.seconds)):
            data = self.stream.read(self.chunk)
            if print_once:
                print(data)
                print(data[0])
                print(type(data))
                print(type(data[0]))
                print_once = False
            frames.append(data)
        print(type(frames))
        print(type(frames[0]))

        self.publish(frames)


    def publish(self, frames):
        # msg = AudioData()
        msg = AudioDataStringArray()
        msg.data = frames
        self.frame_pub.publish(msg)
        

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

    node = RosPyaudio(pub_hz)
    while not rospy.is_shutdown():
        node.getAudioFrames()
        rate.sleep()
    
    node.terminateStream()

if __name__=="__main__":
    main()