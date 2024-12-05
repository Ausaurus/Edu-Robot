#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def camera_publisher():
    rospy.init_node('camera_publisher')
    pub = rospy.Publisher('camera_image', Image, queue_size=1)  
    bridge = CvBridge()
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        rospy.logerr("Error opening camera")
        return

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to capture image")
            continue

        try:
            cropped_frame = frame[150:480, 200:500]
            pub.publish(bridge.cv2_to_imgmsg(cropped_frame, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as ex:
            rospy.logerr(f"Unexpected error: {ex}")

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("Camera publisher node terminated.")
