#!/home/manfred/catkin_ws/src/.tts/bin/python3
import rospy
from std_msgs.msg import String
from google.cloud import texttospeech
from google.oauth2 import service_account #type: ignore
import os
import io
import pygame  #type: ignore

SERVICE_ACCOUNT_FILE = '/home/manfred/catkin_ws/src/edu_robot_pkg/src/text_to_speech_youtube_2.json'

def google_tts(text):
    """Generate speech audio from text using Google Cloud TTS."""
    credentials = service_account.Credentials.from_service_account_file(SERVICE_ACCOUNT_FILE)
    client = texttospeech.TextToSpeechClient(credentials=credentials)

    input_text = texttospeech.SynthesisInput(text=text)
    voice = texttospeech.VoiceSelectionParams(
        language_code="en-US",
        ssml_gender=texttospeech.SsmlVoiceGender.NEUTRAL
    )
    audio_config = texttospeech.AudioConfig(
        audio_encoding=texttospeech.AudioEncoding.MP3
    )

    response = client.synthesize_speech(
        input=input_text, voice=voice, audio_config=audio_config
    )
    return response.audio_content

def play_audio(audio_content):
    """Play the synthesized audio."""
    pygame.mixer.init()
    
    temp_audio_file = "/tmp/tts_audio.mp3"
    with open(temp_audio_file, "wb") as f:
        f.write(audio_content)
    
    pygame.mixer.music.load(temp_audio_file)
    pygame.mixer.music.play()
    
    while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)

def callback(data):
    """Callback for ROS Subscriber."""
    text = data.data
    rospy.loginfo(f"Received text for TTS: {text}")
    
    rospy.loginfo("Generating speech...")
    audio_content = google_tts(text)
    rospy.loginfo("Playing speech...")
    play_audio(audio_content)

if __name__ == '__main__':
    rospy.init_node('tts_node')

    try:
        rospy.Subscriber('tts_input', String, callback)
        rospy.loginfo("Google TTS node started. Listening for input...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down Google TTS node.")



