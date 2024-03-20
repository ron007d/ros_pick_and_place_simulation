#!/usr/bin/env python
'''
Create by - Bishwajit Kumar Poddar
'''
import sys
import os
sys.path.append(os.path.dirname(__file__))


import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
from calibirate_position import calibirator
import json



class Object_detector:
    
    def __init__(self) -> None:
        self.calibiration = calibirator()
        


    def callback(self,data):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(data,"bgr8")
        # cv_image = cv2.transpose(cv_image)
        
        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define red color ranges
        lower_red = np.array([0, 220, 220])
        upper_red = np.array([0, 255, 255])

        # Create a mask for red color
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Filter contours based on area
            if cv2.contourArea(contour) > 100:
                # Get bounding rectangle and calculate center and angle
                x, y, w, h = cv2.boundingRect(contour)
                center = (x + w // 2, y + h // 2)
                
                
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int64(box)
                angle = rect[-1]
                # angle = 0  # You need to calculate the angle based on the contour orientation
                
                # Draw rectangle around detected object
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Draw circle at center
                cv2.circle(image, center, 5, (0, 0, 255), -1)
                
                location = self.calibiration.get_location_from_pixel(center[0],center[1])
                # Display angle
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(image, f"Angle: {angle:.2f}", (x, y), font, 0.5, (255, 255, 255), 2)
                cv2.putText(image, f"location: {location}", (x, y + h + 20), font, 0.5, (255, 255, 255), 2)
                pub = rospy.Publisher('object_location',String,queue_size=10)
                publishing_data = { 'angel' : angle,
                        'position':{
                            "x" : location[0],
                            "y": location[1]
                        }}
                pub.publish(json.dumps(publishing_data))
        
        cv2.namedWindow('Live Video')
        cv2.imshow('Live Video',image)
        cv2.waitKey(1)
        

    def listener(self):
        rospy.init_node('image_viewer',anonymous= True)
        
        rospy.Subscriber('/camera1/image_raw',Image, self.callback)
        
        rospy.spin()



# Testing and current Running
    
if __name__ == '__main__':
    x = Object_detector()    
    x.listener()