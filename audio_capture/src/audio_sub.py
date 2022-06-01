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

from audio_common_msgs.msg import AudioData
from audio_common_msgs.msg import AudioDataStringArray

class RosPyaudio():
    def __init__(self):
        self.chunk          = 1024  # Record in chunks of 1024 samples
        self.sample_format  = pyaudio.paInt16  # 16 bits per sample
        self.channels       = 8
        self.fs             = 44100  # Record at 44100 samples per second
        self.seconds        = 3
        self.p = pyaudio.PyAudio()  # Create an interface to PortAudio
        self.stream = self.p.open(format=self.sample_format,
                                channels=self.channels,
                                rate=self.fs,
                                frames_per_buffer=self.chunk,
                                input=True)
        self.ext_frames  = []
        self.frame_count = 0


        self.frame_sub = rospy.Subscriber("audio/all_channels", AudioDataStringArray, self.audioSubCallback)
        
        
    def setHz(self, hz):
        self.seconds = 1 / hz

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


    def publish(self, frames):
        msg = AudioData()
        msg.data = frames
        self.frame_pub.publish(msg)
        

    def terminateStream(self):
        # Stop and close the stream 
        self.stream.stop_stream()
        self.stream.close()
        # Terminate the PortAudio interface
        self.p.terminate()
        print('Node terminated')

    def audioSubCallback(self, msg):
        frames = msg.data
        self.frame_count += 1
        self.concatFrames(frames)

    def concatFrames(self, frames):
        self.ext_frames += frames
        if self.frame_count > 10:
            self.saveFile(self.ext_frames)
            self.ext_frames  = []
            self.frame_count = 0


    def saveFile(self, frames):
        # Save the recorded data as a WAV file
        file_name = 'pyaudio_saved_2.wav'
        wf = wave.open(file_name, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(self.p.get_sample_size(self.sample_format))
        wf.setframerate(self.fs)
        wf.writeframes(b''.join(frames))
        wf.close()
        print('save: '+file_name)

def main():
    rospy.init_node('audio_sub')

    pub_hz = 10
    rate   = rospy.Rate(pub_hz) # 10hz

    node = RosPyaudio()
    node.setHz(pub_hz)
    while not rospy.is_shutdown():
        node.getAudioFrames()
        rate.sleep()
    
    node.terminateStream()

if __name__=="__main__":
    main()