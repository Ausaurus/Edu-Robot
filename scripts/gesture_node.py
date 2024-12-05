#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import mediapipe as mp
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import cv2

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

bridge = CvBridge()

last_time = 0

def gesture_recog(landmarks):
    if landmarks[4].x > landmarks[3].x and landmarks[8].y > landmarks[6].y and landmarks[12].y > landmarks[10].y and landmarks[16].y > landmarks[14].y and landmarks[20].y > landmarks[18].y and landmarks[2].x < landmarks[0].x:
        return "pause"
    elif landmarks[4].y < landmarks[3].y and landmarks[8].x > landmarks[6].x and landmarks[12].x > landmarks[10].x and landmarks[16].x > landmarks[14].x and landmarks[20].x > landmarks[18].x:
        return "volume up"
    elif landmarks[4].y > landmarks[3].y and landmarks[8].x > landmarks[6].x and landmarks[12].x > landmarks[10].x and landmarks[16].x > landmarks[14].x and landmarks[20].x > landmarks[18].x:
        return "volume down"
    elif landmarks[4].y < landmarks[3].y and landmarks[8].x > landmarks[6].x and landmarks[12].x < landmarks[10].x and landmarks[16].x < landmarks[14].x and landmarks[20].x < landmarks[18].x and landmarks[8].x > landmarks[0].x:
        return "skip"
    elif landmarks[4].y < landmarks[3].y and landmarks[8].x < landmarks[6].x and landmarks[12].x > landmarks[10].x and landmarks[16].x > landmarks[14].x and landmarks[20].x > landmarks[18].x and landmarks[8].x < landmarks[0].x:
        return "rewind"
    elif landmarks[4].x < landmarks[3].x and landmarks[8].y < landmarks[6].y and landmarks[12].y < landmarks[10].y and landmarks[16].y < landmarks[14].y and landmarks[20].y < landmarks[18].y and landmarks[2].x < landmarks[0].x:
        return "resume"
    else:
        return "no gesture detected"

def process(frame):
    global last_time
    current_time = rospy.get_time()
    image = bridge.imgmsg_to_cv2(frame, "rgb8")
    image = cv2.flip(image, 1)
    with mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.7) as hands:
        results = hands.process(image)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                if current_time - last_time >= 1.0: # Reduce rate to 1Hz
                    pub.publish(gesture_recog(hand_landmarks.landmark))
                    print(gesture_recog(hand_landmarks.landmark))
                    last_time = current_time
            

if __name__ == '__main__':
    rospy.init_node('gesture_recog_node')
    pub = rospy.Publisher('gesture_detected', String, queue_size=1)
    rospy.Subscriber('camera_image', Image, process, queue_size=1)
    rospy.spin()



