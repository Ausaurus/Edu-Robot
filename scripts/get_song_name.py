#!/home/manfred/catkin_ws/src/.tts/bin/python3
import rospy
from std_msgs.msg import String
import pyaudio # type: ignore
import wave
import threading
from sound_play.libsoundplay import SoundClient

FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1024

stop_recording = False
recording_prompt = False
last_time = 0

audio = pyaudio.PyAudio()

speech = SoundClient()

def record_audio():
    global stop_recording
    frames = []

    print("Recording... Press 's' again to stop.")

    stream = audio.open(format=pyaudio.paInt16,
                    channels=1,            
                    rate=16000,           
                    input=True,
                    frames_per_buffer=1024)

    try:
        while not stop_recording:
            data = stream.read(CHUNK)
            frames.append(data)
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

def monitor_keyboard():
    global stop_recording
    stop_recording = True

def recording(msg):
    rospy.loginfo(f"Callback triggered with data: {msg.data}")
    global recording_prompt
    recording_prompt = True

def recording_stop(msg):
    global stop_recording
    stop_recording = True
    
def audio_publisher(msg):
    global recording_prompt
    global stop_recording
    pub = rospy.Publisher('/song_topic', String, queue_size=1)
    audio_file_path = '/home/manfred/catkin_ws/src/edu_robot_pkg/audio/songname.wav'
    if recording_prompt:
        print('start recording song name')
        stop_recording = False
        recording_prompt = False
        frames = record_audio()
        save_audio(frames, audio_file_path)
        print(f"Audio saved as {audio_file_path}")
        pub.publish(audio_file_path)
        rospy.loginfo(f"Published new audio file path: {audio_file_path}")
        last_time = rospy.get_time()

if __name__ == '__main__':
    try:
        rospy.Subscriber('ini_music', String, audio_publisher)
        rospy.Subscriber('recording_start', String, recording)
        rospy.Subscriber('music_stop', String, recording_stop)
        rospy.init_node("get_song_node")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    finally:
        audio.terminate()

