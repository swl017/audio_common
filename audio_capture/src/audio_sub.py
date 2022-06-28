#!/usr/bin/env python
'''
@file   audio_sub.py
@brief  Subscribe to audio messages and save to wave files.
@date   2022-06-01
@author Seungwook Lee, USRG@KAIST
@todo   Speedup audio/chx pub rate by changing where splitChannels is executed
@todo   Minimize size of published data per channel(It's almost as large as a video stream now(>2MB/s)!)
'''
import rospy
import pyaudio
import wave
import numpy as np
import os

from audio_common_msgs.msg import AudioDataStringArray # A custom message for this use case
from audio_common_msgs.msg import AudioInfo

class WavAudioSub():
    def __init__(self):
        # Default values. Subject to be changed by audio/all/info topic
        self.sample_format = pyaudio.paInt16
        self.channels      = 8
        self.fs            = 44100              # Record at 44100 samples per second
        self.coding_format = '.wav'
        self.p             = pyaudio.PyAudio()  # Create an interface to PortAudio
        self.oldest_msg_time = None
        self.ext_frames    = []
        self.frame_count   = 0
        self.file_name     = 'wave_'+ str(self.channels) + '_' + str(self.sample_format) + '_' + str(self.fs) + '_'
        
        #### Parameters ####
        self.file_length_sec = 30.0              # Pick one of the two: time length(default) vs frame count
        self.frame_count_thres = 10             # 10 frames corresponding to 1 second
        self.file_path     = os.getcwd() + '/'

        self.frame_sub     = rospy.Subscriber("audio/all", AudioDataStringArray, self.audioSubCallback)
        self.info_sub      = rospy.Subscriber("audio/all/info", AudioInfo, self.infoSubCallback)
        self.channel_pub   = [rospy.Publisher("audio/ch"+str(i), AudioDataStringArray, queue_size=1) for i in range(self.channels)]
        
        print('Node initiated: audio_sub')
        print('file directory: ' + self.file_path)

    def infoSubCallback(self, msg):
        # Sync to audio_pub node's setting
        self.sample_format = int(msg.sample_format)
        self.channels      = msg.channels
        self.fs            = msg.sample_rate
        self.file_name     = 'wave_'+ str(self.channels) + '_' + str(self.sample_format) + '_' + str(self.fs) + '_'
        
    def audioSubCallback(self, msg):
        if self.oldest_msg_time is None:
            self.oldest_msg_time = rospy.Time.now()
        self.frame_count += 1
        self.concatFrames(msg)

    def concatFrames(self, msg):
        frames = msg.data
        self.ext_frames += frames

        # Save file and split channels
        time_since_oldest = rospy.Time.now() - self.oldest_msg_time
        if time_since_oldest.to_sec() > self.file_length_sec: # interval by time
        # if self.frame_count > self.frame_count_thres: # interval by numbers of received messages
            # time = msg.header.stamp # timestamp at the end of the sound
            time = self.oldest_msg_time # timestamp at the start of the sound
            self.save_time = str(time.secs * 10**9 + time.nsecs)
            try:
                self.saveFile(self.ext_frames)
                for i in range(self.channels):
                    ch_data = self.splitChannels(self.ext_frames, i)
                    self.savePerChannel(ch_data, i)
                    self.pubPerChannel(ch_data, time, i)
            except:
               print("[Exception] Failed to save files. \
                      Please make sure you are working with correct audio parameters, \
                      or subscribing to audio/all/info topic")
               pass
            self.ext_frames  = []
            self.frame_count = 0
            self.oldest_msg_time = None

    def saveFile(self, frames):
        # Save the recorded data as a WAV file
        wf = wave.open(self.file_name + self.save_time + self.coding_format, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(self.p.get_sample_size(self.sample_format))
        wf.setframerate(self.fs)
        wf.writeframes(b''.join(frames))
        wf.close()
        print('saved: '+self.file_name + self.save_time + self.coding_format)

    def splitChannels(self, sdata, channel):
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

    def savePerChannel(self, ch_data, channel):
        # Save channel to a separate file
        file_name = self.file_name + self.save_time + '_' + str(channel) + self.coding_format
        outwav = wave.open(file_name, 'w')
        outwav.setnchannels(1)
        outwav.setsampwidth(self.p.get_sample_size(self.sample_format)) # bytes per sample
        outwav.setframerate(self.fs)
        outwav.writeframes(ch_data.tostring())
        outwav.close()
        print('saved: '+file_name)

    def pubPerChannel(self, ch_data, time, channel):
        msg = AudioDataStringArray()
        msg.header.stamp = time
        msg.data         = ch_data.tostring()
        self.channel_pub[channel].publish(msg)


def main():
    rospy.init_node('audio_sub')

    node = WavAudioSub()
    rospy.spin()

if __name__=="__main__":
    main()
