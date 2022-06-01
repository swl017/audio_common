#!/usr/bin/env python
'''
@file   pyaudio_sub8.py
@brief  Subscribe to audio messages and save to wave files.
@date   2022-06-01
@author Seungwook Lee, USRG@KAIST
'''
import rospy
import pyaudio
import wave
import numpy as np

from audio_common_msgs.msg import AudioData
from audio_common_msgs.msg import AudioDataStringArray

class RosPyaudio():
    def __init__(self):
        self.sample_format  = pyaudio.paInt16  # 16 bits per sample
        self.channels       = 8
        self.fs             = 44100  # Record at 44100 samples per second
        self.p = pyaudio.PyAudio()  # Create an interface to PortAudio
        self.ext_frames  = []
        self.frame_count = 0
        self.file_name   = None

        self.frame_sub = rospy.Subscriber("audio/all_channels", AudioDataStringArray, self.audioSubCallback)
        
    def audioSubCallback(self, msg):
        self.frame_count += 1
        self.concatFrames(msg)

    def concatFrames(self, msg):
        time = msg.header.stamp.secs * 10**9 + msg.header.stamp.nsecs
        frames = msg.data
        self.ext_frames += frames
        if self.frame_count > 10:
            self.saveFile(self.ext_frames, time)
            for i in range(self.channels):
                ch_data = self.splitChannels(self.ext_frames, time, i)
                self.savePerChannel(ch_data, time, i)
            self.ext_frames  = []
            self.frame_count = 0

    def saveFile(self, frames, time):
        # Save the recorded data as a WAV file
        self.file_name = 'pyaudio_saved_2_' + str(time)
        wf = wave.open(self.file_name + '.wav', 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(self.p.get_sample_size(self.sample_format))
        wf.setframerate(self.fs)
        wf.writeframes(b''.join(frames))
        wf.close()
        print('save: '+self.file_name + '.wav')

    def splitChannels(self, sdata, time, channel):
        depth = self.p.get_sample_size(self.sample_format)
        # Extract channel data (24-bit data not supported)
        typ = { 1: np.uint8, 2: np.uint16, 4: np.uint32 }.get(depth)
        if not typ:
            raise ValueError("sample width {} not supported".format(depth))
        if channel >= self.channels:
            raise ValueError("cannot extract channel {} out of {}".format(channel+1, self.channels))
        # print ("Extracting channel {} out of {} channels, {}-bit depth".format(channel+1, self.channels, depth*8))
        data = np.fromstring(''.join(sdata), dtype=typ)

        return data[channel::self.channels]

    def savePerChannel(self, ch_data, time, channel):
        # Save channel to a separate file
        file_name = self.file_name + '_' + str(channel) + '.wav'
        outwav = wave.open(file_name, 'w')
        outwav.setnchannels(1)
        outwav.setsampwidth(self.p.get_sample_size(self.sample_format)) # bytes per sample
        outwav.setframerate(self.fs)
        outwav.writeframes(ch_data.tostring())
        outwav.close()
        print('saved file: '+file_name)


def main():
    rospy.init_node('audio_sub')

    node = RosPyaudio()
    rospy.spin()

if __name__=="__main__":
    main()