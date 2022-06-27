'''
source: https://realpython.com/playing-and-recording-sound-python/#pyaudio_1
'''

import pyaudio
import wave
from audio_common_msgs.msg import AudioData

chunk = 1024  # Record in chunks of 1024 samples
sample_format = pyaudio.paInt16  # 16 bits per sample
channels = 8
fs = 16000  # Record at 44100 samples per second
seconds = 3
filename = "output.wav"

p = pyaudio.PyAudio()  # Create an interface to PortAudio

print('Recording')

stream = p.open(format=sample_format,
                channels=channels,
                rate=fs,
                frames_per_buffer=chunk,
                input=True,
                input_device_index=24)

frames = []  # Initialize array to store frames

# Store data in chunks for 3 seconds
print_once = True
for i in range(0, int(fs / chunk * seconds)):
    data = stream.read(chunk)
    # if print_once:
    #     print(data)
    #     print(data[0])
    #     print(type(data))
    #     print(type(data[0]))
    #     print_once = False
    frames.append(data)
# print(type(frames))
# print(type(frames[0]))
# fdfd = AudioData()
# fdfd.data = frames
# print(type(fdfd.data))

# Stop and close the stream 
stream.stop_stream()
stream.close()
# Terminate the PortAudio interface
p.terminate()

print('Finished recording')

# Save the recorded data as a WAV file
wf = wave.open(filename, 'wb')
wf.setnchannels(channels)
wf.setsampwidth(p.get_sample_size(sample_format))
wf.setframerate(fs)
wf.writeframes(b''.join(frames))
wf.close()