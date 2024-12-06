#!/home/manfred/catkin_ws/src/.tts/bin/python3

import io
from google.oauth2 import service_account #type: ignore
from google.cloud import speech
import rospy
from std_msgs.msg import String

client_file = '/home/manfred/catkin_ws/src/edu_robot_pkg/src/speech_to_text_new.json'

credentials = service_account.Credentials.from_service_account_file(client_file)
client = speech.SpeechClient(credentials=credentials)

def get_transcribe_text(audio_file):
	audio_file = '/home/manfred/catkin_ws/src/edu_robot_pkg/audio/output.wav'
	with io.open(audio_file, 'rb') as f:
		content = f.read()
		audio = speech.RecognitionAudio(content=content)
		config = speech.RecognitionConfig(
			encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
			sample_rate_hertz=48000,
			language_code='en-US'
		)
		response = client.recognize(config=config, audio=audio)
	for result in response.results:
		return result.alternatives[0].transcript
	return "Error: No transcription available"

def get_song_name(audio_file):
	audio_file = '/home/manfred/catkin_ws/src/edu_robot_pkg/audio/songname.wav'
	with io.open(audio_file, 'rb') as f:
		content = f.read()
		audio = speech.RecognitionAudio(content=content)
		config = speech.RecognitionConfig(
			encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
			sample_rate_hertz=48000,
			language_code='en-US'
		)
		response = client.recognize(config=config, audio=audio)
	for result in response.results:
		return result.alternatives[0].transcript
	return "Error: No transcription available"


def callback(data):
	question = get_transcribe_text(data.data)
	question = question.lower().strip()
	if question == "music":
		pub_music.publish(question)
	else:
		rospy.loginfo(f"Received question: {question}")
		pub.publish(question)

def pub_song_name(data):
	songname = get_song_name(audio_file=data.data)
	songname = songname.lower().strip()
	pub_songname.publish(songname)

if __name__ == "__main__":
	rospy.init_node('speech_to_text')
	pub_music = rospy.Publisher("ini_music", String, queue_size=10)
	pub_songname = rospy.Publisher("song_name", String, queue_size=10)
	pub = rospy.Publisher('student_question', String, queue_size=10)
	rospy.Subscriber('/audio_topic', String, callback)
	rospy.Subscriber("song_topic", String, pub_song_name)
	rospy.spin()



