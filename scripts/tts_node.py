#!/home/manfred/catkin_ws/src/.tts/bin/python3
import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
import time
import pyaudio# type: ignore
import wave

def callback(data):
    text = data.data
    rospy.loginfo(f"TTS Output: {text}")    
    soundhandle.say(text)

def put_the_phone_down(data):
    CHUNK = 1024
    audio = pyaudio.PyAudio()
    f = wave.open(r"/home/manfred/catkin_ws/src/edu_robot_pkg/audio/PUT THE PHONE DOWN.wav","rb")
    stream = audio.open(format = audio.get_format_from_width(f.getsampwidth()),  
                channels = f.getnchannels(),  
                rate = f.getframerate(),  
                output = True)  
    data = f.readframes(CHUNK)  
    while data:  
        stream.write(data)  
        data = f.readframes(CHUNK)  
    stream.stop_stream()  
    stream.close()  
    audio.terminate() 


if __name__ == '__main__':
    rospy.init_node('tts_node')
    soundhandle = SoundClient()
    time.sleep(1)
    rospy.loginfo("SoundClient initialized successfully.")
    rospy.Subscriber('tts_input', String, callback)
    rospy.Subscriber('Phone_detection', String, put_the_phone_down)
    rospy.spin()

