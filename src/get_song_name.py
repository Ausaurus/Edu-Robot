#!/home/manfred/catkin_ws/src/.tts/bin/python3
import rospy
from std_msgs.msg import String
import pyaudio #type: ignore
import wave
import webrtcvad #type: ignore

# Audio configuration
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 48000
CHUNK = 480

# Global flag to control recording
stop_recording = False
recording_prompt = False
last_time = 0

# Initialize PyAudio
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

# Function to record audio
def record_audio():
    global stop_recording
    frames = []
    silent_chunks = 0
    max_silent_chunks = int(RATE / CHUNK * 2)

    print("Recording... Speak to start and stop recording automatically.")

    # Open audio stream
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

# Function to save audio to a file
def save_audio(frames, filename):
    wf = wave.open(filename, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(audio.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

# Thread to listen for the stop key
def monitor_keyboard():
    global stop_recording
    stop_recording = True

def recording(msg):
    rospy.loginfo(f"Callback triggered with data: {msg.data}")
    global recording_prompt
    recording_prompt = True
    
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
        rospy.init_node("get_song_node")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    finally:
        # Terminate PyAudio
        audio.terminate()

