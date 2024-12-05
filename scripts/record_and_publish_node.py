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
music_recording = False
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

def music_recording_ini(msg):
    global music_recording
    music_recording = True

def music_recording_deact(msg):
    global music_recording
    music_recording = False

def recording(msg):
    rospy.loginfo(f"Callback triggered with data: {msg.data}")
    global recording_prompt
    recording_prompt = True

def recording_stop(msg):
    global stop_recording
    stop_recording = True
    
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
                    speech.say('start recording')
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
        rospy.init_node('record_and_publish_node')
        rospy.Subscriber('recording_start', String, recording)
        rospy.Subscriber('recording_stop', String, recording_stop)
        rospy.Subscriber('initiate', String, audio_publisher)
        rospy.Subscriber('ini_music', String, music_recording_ini) 
        rospy.Subscriber('/song_topic', String, music_recording_deact) 
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        audio.terminate()

