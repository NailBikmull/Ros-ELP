#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy as np

class MotionDetector:
    def __init__(self):
        self.prev_frame = None
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/usb_cam/motion_detection", Bool, queue_size=10)
        
    def callback(self, msg):
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
            
        detected = Bool()
        
        if self.prev_frame is not None:
            diff = cv2.absdiff(gray, self.prev_frame)
            if np.sum(diff) > 1500000:  
                rospy.loginfo("Motion detected!")
                detected.data = True
            else:
                detected.data = False
                rospy.loginfo("Motion not detected!")
            self.pub.publish(detected)
            
        self.prev_frame = gray
            


if __name__ == "__main__":
    rospy.init_node('detector', anonymous=False)
    detector = MotionDetector()
    rospy.Subscriber("/usb_cam/image_raw", Image, detector.callback)
    rospy.spin()