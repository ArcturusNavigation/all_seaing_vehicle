#!/usr/bin/env python3

# sound signal detection for roboboat task 6 (harbor alert)
# basically just listens to the mic and tries to figure out if we're hearing
# one of the competition frequencies (600, 800, or 1000 hz)
# then counts how many self.blasts there are (1 or 2) and prints what to do
#
# put in a few dependencies
#

import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String

import pyaudio
import numpy as np
import time
import wave
from scipy.io.wavfile import write
import sounddevice

# OUT_CHANNELS = 1
# OUT_DEVICE_INDEX = 4

SAMPLE_RATE = 44100
CHUNK = 4096  # number of frames per read

TARGET_FREQS = [600.0, 800.0, 1000.0]
# the 3 possible frequencies from the competition rules

class SoundSignalDetector(Node):
    def __init__(self):
        super().__init__("sound_signal_detector")

        global CHUNK, SAMPLE_RATE
        FORMAT = pyaudio.paFloat32
        CHANNELS = 1  # mono mic
        DEVICE_INDEX = 8
        
        self.declare_parameter("target_freq", 600.0)
        global TARGET_FREQS
        TARGET_FREQS = [self.get_parameter("target_freq").get_parameter_value().double_value]

        self.timer = self.create_timer(1/float(SAMPLE_RATE), self.timer_callback)

        self.harbor_publisher = self.create_publisher(String, "harbor_detect", 10)

        # setting up the microphone
        self.p = pyaudio.PyAudio()
        SAMPLE_RATE = int(self.p.get_device_info_by_index(DEVICE_INDEX)["defaultSampleRate"])
        print(f'Sample rate: {SAMPLE_RATE}')
        self.stream = self.p.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=SAMPLE_RATE,
            # rate=int(p.get_device_info_by_index(DEVICE_INDEX)["defaultSampleRate"]),
            input=True,
            frames_per_buffer=CHUNK,
            input_device_index=DEVICE_INDEX,
        )

        #variables we gotta check for, that we call other functions for and have to observe
        self.blasts = 0
        self.detected_frequency = None
        self.hearing_tone = False
        self.tone_start = None
        self.silence_start = None

    def timer_callback(self):
        # these will probably need tuning
        POWER_THRESHOLD = 0.8   # how loud the fft peak needs to be to count as a real tone
        MIN_BLAST_TIME = 0.2    # blast has to be at least this long (seconds) otherwise its just noise
        MIN_SILENCE_TIME = 0.2 # if less than this ignore gap, probably missed detecting in between
        MAX_DOUBLE_SILENCE_TIME = 0.7
        SILENCE_TIMEOUT = 2.5   # how long we wait after first blast to see if theres a second one

        global CHUNK, SAMPLE_RATE
        #running FFT to get our loudest frequency
        data = self.stream.read(CHUNK, exception_on_overflow=False)
        # print(data)
        freqency, magnitude = self.get_peak_frequency(data, SAMPLE_RATE) 

        matched = None #check to see if we match one of the freqencies
        if magnitude > POWER_THRESHOLD:
            matched = self.check_if_target(freqency)

        
        now = time.time()

        if matched is not None: #we have to start the timer
            if self.hearing_tone == False:
                self.hearing_tone = True
                self.tone_start = now
                self.detected_frequency = matched
                if self.silence_start is not None:
                    if now - self.silence_start > MIN_SILENCE_TIME and now-self.silence_start < MAX_DOUBLE_SILENCE_TIME:
                        # double
                        self.harbor_publisher.publish(String(data=f"{int(self.detected_frequency)}_double"))
                        self.blasts = 0
                        self.detected_frequency = None
                        self.hearing_tone = False
                        self.tone_start = None
                        self.silence_start = None
                        return
            self.silence_start = None

        else: #make sure that we're counting to see if we got a valid sound duration sample
            if self.hearing_tone == True:
                duration = now - self.tone_start
                self.hearing_tone = False #writing down how long the sound was basically

                if duration >= MIN_BLAST_TIME: #if we got within the blast time
                    #make sure to add the margin of error
                    self.blasts += 1
                    self.silence_start = now
                    if self.blasts >= 3:
                        # single
                        self.harbor_publisher.publish(String(data=f"{int(self.detected_frequency)}_single"))
                        self.blasts = 0
                        self.detected_frequency = None
                        self.hearing_tone = False
                        self.tone_start = None
                        self.silence_start = None
                        return
                else:
                    self.tone_start = None #if we're getting random noise just don't return it

            if self.silence_start is not None: #waiting to see if we get another blast
                if now - self.silence_start >= SILENCE_TIMEOUT:
                    # single
                    self.harbor_publisher.publish(String(data=f"{int(self.detected_frequency)}_single"))
                    self.blasts = 0
                    self.detected_frequency = None
                    self.hearing_tone = False
                    self.tone_start = None
                    self.silence_start = None
                    return


    def get_peak_frequency(self, audio_data, sr):
        # convert raw bytes to numpy array
        samples = np.frombuffer(audio_data, dtype=np.float32).astype(np.float64)

        # self.get_logger().info(samples)
        # self.get_logger().info(np.max(samples))

        #to build indces to the actual frequencies
        freqencies = np.fft.rfftfreq(len(samples), d=1.0 / sr)
        # to tell us what the energies are 
        spectrum = np.abs(np.fft.rfft(samples))

        spectrum = spectrum / np.linalg.norm(spectrum)

        #so frequencies and spectrum each give at the beginning of each index the frequency and strength

        #I tried to make a dictionary for all these values but realized that's just too much work

        # find whichever frequency has the most energy/is the strongest so we can tell that there's an alert basically
        peak_frequency = np.argmax(spectrum)
        # self.get_logger().info(f'{freqencies[peak_frequency], spectrum[peak_frequency]}')
        # print(spectrum)
        # print(np.max(spectrum))
        return freqencies[peak_frequency], spectrum[peak_frequency]


    #checking to see if the detected frequency is close enough to one of our target ones
    def check_if_target(self, freqency):
        FREQ_TOLERANCE = 0.1  # it says +/- 5% on the frequencies but idk if we need to adjust it more

        global TARGET_FREQS
        for t in TARGET_FREQS:
            check = abs(freqency - t)
            if check / t <= FREQ_TOLERANCE:
                return t
            
    def destroy_node(self):
        self.stream.close()
        self.p.terminate() #have to close our microphone
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SoundSignalDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
