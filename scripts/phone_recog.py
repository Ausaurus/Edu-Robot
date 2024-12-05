#!/home/manfred/python_OpenCV/.yolov8_env/bin/python3
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
import numpy as np
from std_msgs.msg import String
import logging 
from ultralytics import YOLO  # type: ignore


bridge = CvBridge()

model = YOLO("yolo11n.pt")

pub = rospy.Publisher('Phone_detection', String, queue_size=1)

class_name = "cell phone"
double_check = False
timer_active = False


def phone_detection(frame):
    results = model(frame)
    for result in results:
        for class_id in result.boxes.cls:
            class_id = int(class_id)
            if class_id == 67:
                return True  
    return False  

def image_callback(msg):
    global double_check, timer_active, pause_duration

    if timer_active:
        return

    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"Error converting image: {e}")
        return

    phone_found = phone_detection(frame)

    if phone_found:
        if not double_check:
            rospy.loginfo("WARNING: PUT THE PHONE DOWN")
            timer_active = True
            pause_duration = 5
            rospy.Timer(rospy.Duration(pause_duration), reset_timer, oneshot=True)
            double_check = True
        else:
            rospy.loginfo("PUT THE PHONE DOWN!!!!!!!")
            pub.publish("Phone detected")
            timer_active = True
            pause_duration = 6
            rospy.Timer(rospy.Duration(pause_duration), reset_timer, oneshot=True)
            double_check = False

def reset_timer(event):
    global timer_active
    timer_active = False

if __name__ == "__main__":   
    logging.getLogger().setLevel(logging.WARNING)  # Suppress info/debug messages
    rospy.init_node('Phone_recog_node')
    rospy.Subscriber('camera_image', Image, image_callback)
    rospy.spin()
        
        