#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

class ColorFilter:
    def __init__(self):
        self.color_option = None
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/usb_cam/image_musk", Image, queue_size=10)
        
        cv2.namedWindow('Original Image', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Filtered Image', cv2.WINDOW_NORMAL)

    def get_color(self):
        rospy.set_param("color", "red")
        self.color_option = rospy.get_param("color")
        print(self.color_option)
        
    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            image_rgb = cv_image.copy()
            image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
            
            image_masked = np.zeros_like(image_rgb)  
            
            if self.color_option == "red":
                H_min = 0
                H_max = 10
                S_min = 0
                S_max = 255
                V_min = 0
                V_max = 255
                mask = cv2.inRange(image_hsv, (H_min, S_min, V_min), (H_max, S_max, V_max))
                image_masked = cv2.bitwise_and(image_rgb, image_rgb, mask=mask)

            elif self.color_option == "green":
                H_min = 35
                H_max = 95
                S_min = 0
                S_max = 255
                V_min = 0
                V_max = 255
                mask = cv2.inRange(image_hsv, (H_min, S_min, V_min), (H_max, S_max, V_max))
                image_masked = cv2.bitwise_and(image_rgb, image_rgb, mask=mask)

            elif self.color_option == "blue":
                H_min = 110
                H_max = 130
                S_min = 0
                S_max = 255
                V_min = 0
                V_max = 255
                mask = cv2.inRange(image_hsv, (H_min, S_min, V_min), (H_max, S_max, V_max))
                image_masked = cv2.bitwise_and(image_rgb, image_rgb, mask=mask)
            
            filtered_msg = self.bridge.cv2_to_imgmsg(image_masked, "rgb8")
            self.pub.publish(filtered_msg)
            
            cv2.imshow('Original Image', cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
            cv2.imshow('Filtered Image', cv2.cvtColor(image_masked, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr(f"Error in callback: {str(e)}")

if __name__ == "__main__":
    rospy.init_node('color_filter', anonymous=False)
    filter_node = ColorFilter() 
    filter_node.get_color()
    rospy.Subscriber("/usb_cam/image_raw", Image, filter_node.callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()