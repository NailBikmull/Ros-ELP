#!/usr/bin/env python
import rospy
import os
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
global image_rgb

def callback(msg):
    rospy.loginfo("Image has been recived")
    cv_bridge = CvBridge()
    image = cv_bridge.imgmsg_to_cv2(msg)
    image_rgb = cv2.cvtColor(image, cv2.COLOR_YUV2RGB_Y422)
    rgb_msg = cv_bridge.cv2_to_imgmsg(image_rgb, "rgb8")
    rospy.Publisher.publish(rgb_msg)

if __name__ == "__main__":
    rospy.init_node('keeper', anonymous=False)
    rospy.Publisher("/usb_cam/image_color", Image)
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    rospy.spin()
