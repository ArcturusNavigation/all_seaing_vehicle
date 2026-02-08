#!/usr/bin/env python3

# sound signal detection for roboboat task 6 (harbor alert)
# basically just listens to the mic and tries to figure out if we're hearing
# one of the competition frequencies (600, 800, or 1000 hz)
# then counts how many blasts there are (1 or 2) and prints what to do
#
# put in a few dependencies
#

import pyaudio
import numpy as np
import time
# import wave
# from scipy.io.wavfile import write
# import sounddevice

SAMPLE_RATE = 44100
CHUNK = 4096  # number of frames per read
FORMAT = pyaudio.paInt16
CHANNELS = 1  # mono mic
DEVICE_INDEX = 8
# OUT_CHANNELS = 1
# OUT_DEVICE_INDEX = 4

# the 3 possible frequencies from the competition rules
TARGET_FREQS = [600.0, 800.0, 1000.0]
FREQ_TOLERANCE = 0.05  # it says +/- 5% on the frequencies but idk if we need to adjust it more

# these will probably need tuning
POWER_THRESHOLD = 1e6   # how loud the fft peak needs to be to count as a real tone
MIN_BLAST_TIME = 0.2    # blast has to be at least this long (seconds) otherwise its just noise
SILENCE_TIMEOUT = 2.0   # how long we wait after first blast to see if theres a second one

# RECORD_SECONDS = 5
# WAVE_OUTPUT_FILENAME = "output.wav"


def get_peak_frequency(audio_data, sr):
    # print(audio_data)
    # convert raw bytes to numpy array
    samples = np.frombuffer(audio_data, dtype=np.int16).astype(np.float64)

    # print(samples)
    # print(np.max(samples))

    #to build indces to the actual frequencies
    freqencies = np.fft.rfftfreq(len(samples), d=1.0 / sr)
    # to tell us what the energies are 
    spectrum = np.abs(np.fft.rfft(samples))

    #so frequencies and spectrum each give at the beginning of each index the frequency and strength

    #I tried to make a dictionary for all these values but realized that's just too much work

    # find whichever frequency has the most energy/is the strongest so we can tell that there's an alert basically
    peak_frequency = np.argmax(spectrum)
    print(freqencies[peak_frequency], spectrum[peak_frequency])
    # print(spectrum)
    # print(np.max(spectrum))
    return freqencies[peak_frequency], spectrum[peak_frequency]


#checking to see if the detected frequency is close enough to one of our target ones
def check_if_target(freqency):
    for t in TARGET_FREQS:
        check = abs(freqency - t)
        if check / t <= FREQ_TOLERANCE:
            return t
        

    return None 



#gotta check for the number of blasts

def listen_for_blasts():
    # setting up the microphone
    p = pyaudio.PyAudio()
    SAMPLE_RATE = int(p.get_device_info_by_index(DEVICE_INDEX)["defaultSampleRate"])
    print(f'Sample rate: {SAMPLE_RATE}')
    stream = p.open(
        format=FORMAT,
        channels=CHANNELS,
        rate=SAMPLE_RATE,
        # rate=int(p.get_device_info_by_index(DEVICE_INDEX)["defaultSampleRate"]),
        input=True,
        frames_per_buffer=CHUNK,
        input_device_index=DEVICE_INDEX,
    )
    #variables we gotta check for, that we call other functions for and have to observe
    blasts = 0
    detected_freqency = None
    hearing_tone = False
    tone_start = None
    silence_start = None

    try:
        while True: #reading some piece of audio
            #running FFT to get our loudest frequency
            data = stream.read(CHUNK, exception_on_overflow=False)
            # print(data)
            freqency, magnitude = get_peak_frequency(data, SAMPLE_RATE) 

            matched = None #check to see if we match one of the freqencies
            if magnitude > POWER_THRESHOLD:
                matched = check_if_target(freqency)

            
            now = time.time()

            if matched is not None: #we have to start the timer
                if hearing_tone == False:
                    hearing_tone = True
                    tone_start = now
                    detected_freqency = matched
                silence_start = None

            else: #make sure that we're counting to see if we got a valid sound duration sample
                if hearing_tone == True:
                    duration = now - tone_start
                    hearing_tone = False #writing down how long the sound was basically

                    if duration >= MIN_BLAST_TIME: #if we got within the blast time
                        #make sure to add the margin of error
                        blasts = blasts + 1
                        silence_start = now
                        if blasts >= 2:
                            break
                    else:
                        tone_start = None #if we're getting random noise just don't return it

                if blasts == 1 and silence_start is not None: #waiting to see if we get another blast
                    if now - silence_start >= SILENCE_TIMEOUT:
                        break

        # print("* recording")

        # frames = []

        # for i in range(0, int(SAMPLE_RATE / CHUNK * RECORD_SECONDS)):
        #         data = stream.read(CHUNK, False)
        #         frames.append(np.fromstring(data, dtype=np.int16))

        # #Convert the list of numpy-arrays into a 1D array (column-wise)
        # numpydata = np.hstack(frames)

        # print("* done recording")

        # # # Save the recorded data as a WAV file
        # # with wave.open(WAVE_OUTPUT_FILENAME, 'wb') as wf:
        # #     wf.setnchannels(CHANNELS)
        # #     wf.setsampwidth(p.get_sample_size(FORMAT))
        # #     wf.setframerate(SAMPLE_RATE)
        # #     wf.writeframes(b''.join(frames))

        # write('test.wav', SAMPLE_RATE, numpydata)


    finally:
        stream.close()
        p.terminate() #have to close our microphone

    return blasts, detected_freqency


if __name__ == "__main__":
    blasts, freqency = listen_for_blasts()
    print(str(blasts) + " blasts at " + str(freqency) + " hz")
