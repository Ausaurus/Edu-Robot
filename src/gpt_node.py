#!/home/manfred/catkin_ws/src/.tts/bin/python3
import rospy
from groq import Groq #type: ignore
from std_msgs.msg import String

client = Groq(api_key='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')

def get_gpt_response(question):
    completion = client.chat.completions.create(
        model="llama3-8b-8192",
        messages=[
            {"role": "system", "content": "You are an edurobot designed to assist students with their learning and answer their questions in a helpful and clear manner."},
            {"role": "user", "content": question},
        ],
        temperature=1,
        max_tokens=50,
        top_p=1,
        stream=True,
    )

    response = ""
    for chunk in completion:
        content = chunk.choices[0].delta.content
        if content is not None:  
            response += content
    return response

def callback(data):
    question = data.data
    rospy.loginfo(f"Received question: {question}")
    answer = get_gpt_response(question)
    rospy.loginfo(f"LLM Response: {answer}")
    pub.publish(answer)

if __name__ == '__main__':
    rospy.init_node('gpt_node')
    pub = rospy.Publisher('tts_input', String, queue_size=10)
    rospy.Subscriber('student_question', String, callback)
    rospy.spin()

