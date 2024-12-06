#!/home/manfred/catkin_ws/src/.tts/bin/python3
import rospy
from std_msgs.msg import String
import pyaudio #type: ignore
import wave
import threading
import webrtcvad #type: ignore
import time

FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 48000
CHUNK = 480

stop_recording = False
music_recording = False
recording_prompt = False
last_time = 0

audio = pyaudio.PyAudio()
vad = webrtcvad.Vad()
vad.set_mode(3)

def is_speech(data):
    frame_duration = 10  
    frame_size = int(RATE * frame_duration / 1000) 
    frame_bytes = frame_size * 2 

    if len(data) < frame_bytes:
        return False

    return vad.is_speech(data[:frame_bytes], RATE)

def record_audio_vad():
    global stop_recording
    frames = []
    silent_chunks = 0
    max_silent_chunks = int(RATE / CHUNK * 2)

    print("Recording... Speak to start and stop recording automatically.")

    stream = audio.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        input_device_index=0,
                        frames_per_buffer=CHUNK)

    try:
        while not stop_recording:
            data = stream.read(CHUNK, exception_on_overflow=False)
            if is_speech(data):
                frames.append(data)
                silent_chunks = 0 
                print("Voice detected, recording...")
            elif frames:
                silent_chunks += 1
                print(f"Silence detected: {silent_chunks}/{max_silent_chunks}")

                if silent_chunks > max_silent_chunks:
                    print("Silence timeout, stopping recording.")
                    break
    finally:
        stream.stop_stream()
        stream.close()

    return frames

def save_audio(frames, filename):
    wf = wave.open(filename, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(audio.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

def music_recording_ini(msg):
    global music_recording
    music_recording = True

def music_recording_deact(msg):
    global music_recording
    music_recording = False

def recording(msg):
    rospy.loginfo(f"Recording triggered with data: {msg.data}")
    global recording_prompt
    recording_prompt = True

def audio_publisher(msg):
    global recording_prompt
    global last_time
    global music_recording
    global stop_recording
    pub = rospy.Publisher('/audio_topic', String, queue_size=10)
    audio_file_path = '/home/manfred/catkin_ws/src/edu_robot_pkg/audio/output.wav'
    current_time = rospy.get_time()
    if current_time - last_time >= 5:
        if not music_recording:
                if recording_prompt:
                    stop_recording = False
                    recording_prompt = False
                    frames = record_audio_vad()
                    save_audio(frames, audio_file_path)
                    print(f"Audio saved as {audio_file_path}")
                    pub.publish(audio_file_path)
                    rospy.loginfo(f"Published new audio file path: {audio_file_path}")
                    last_time = rospy.get_time()

if __name__ == '__main__':
    try:
        rospy.init_node('audio_publisher')
        rospy.Subscriber('recording_start', String, recording)
        rospy.Subscriber('initiate', String, audio_publisher)
        rospy.Subscriber('ini_music', String, music_recording_ini) 
        rospy.Subscriber('/song_topic', String, music_recording_deact) 
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        audio.terminate()


