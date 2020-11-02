#!/usr/bin/env python


## Description:
# This node get_image subscribes to the topics arm_BP and one of the image topics from ZED. When the arm_BP callback signals True, the image callback goes into the function "do_something".

import roslib
roslib.load_manifest('dl_control')
import rospy
import time
import os
import numpy as np
import cv2

from std_msgs.msg import Bool
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray




## Data structure for Image:
# type(data) = class sensor_msgs.msg._Image.Image
# data.header gives the header
# data.height 720 (uint32)
# data.width 1280 (uint32)
# data.encoding bgra8
# data.is_bigendian 0 (False)
# data.step 5120 (uint32)
# data.data: len=3 686 400, type='bytes'

right_pos = False

def paint(pic):
  
# Polygon corner points coordinates 
    pts = np.array([[25, 70], [25, 160],  
                [110, 200], [200, 160],  
                [200, 70], [110, 20]], 
               np.int32)

    cx = 640
    cy = 360
    px = 640 - 200
    py = 360 - 200
    center_coordinates = (cx, cy)
    pixel_coordinates = (px, py)
    radius = 20
    
    pts = pts.reshape((-1, 1, 2)) 
  
    isClosed = True

    color = (255, 0, 0) 
    colorcenter = (0, 255, 0)

    thickness = 2
    image = cv2.circle(pic, center_coordinates, radius, colorcenter, thickness)
    image = cv2.circle(image, pixel_coordinates, radius, color, thickness)

    #image = cv2.polylines(pic, [pts], isClosed, color, thickness) 
    return image, pts

def dnn(pic_data):
    #print("doing some nasty stuff to the picture data") 
    #rospy.loginfo('image_to_dnn -> pic_data is extracting segmentations')
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(pic_data, "bgr8")

    painted, pts = paint(cv_image)
    cv2.imshow("Image window", painted)
    cv2.waitKey(1)

    ## TODO add the actual do_something function
    segm_info = [24.5, 1003, -0.5]
    time.sleep(0.7)
    return segm_info


def callback(data):

    #rospy.loginfo('image_to_dnn -> image callback ')
    segm_pub = rospy.Publisher("dnn_output", Float32MultiArray, queue_size=10)

    #global right_pos
    
    if len(data.data)==3686400 and right_pos == True:
        #print('one frame captured')
            rospy.loginfo('image_to_dnn -> image callback, Right Pos and image use Neural Net')
        
        #while right_pos == True:
           
            segm = dnn(data)
            
            time.sleep(1)
	   
            segments = Float32MultiArray(data=segm)
            segm_pub.publish(segments)
        
     

def callback_bool(data):
    rospy.loginfo('image_to_dnn -> arm_BP callback heard:  %s ', data.data)


    global right_pos

    if data.data==True:
      right_pos = True
    else:
      right_pos = False
    return right_pos



def main():
    
    rospy.init_node('image_to_dnn', anonymous=False)

    image_sub = rospy.Subscriber('/zed2/zed_node/left/image_rect_color',Image,callback)
    
    bp_sub = rospy.Subscriber("arm_BP", Bool, callback_bool)
    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    main()
