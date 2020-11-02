#!/usr/bin/env python


## Description:
# This node get_image subscribes to the topics arm_BP and one of the image topics from ZED. When the arm_BP callback signals True, the image callback goes into the function "do_something".

import roslib
roslib.load_manifest('dl_control')
import rospy
import cv2
import time
import os
import numpy as np

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from dl_control.srv import*

def handle_get_3D_coordinates(req):
    x=1
    y=5
    z=7
    
    return AddTwoIntsResponse(x,y,z)

def add_two_ints_server():
    rospy.init_node('get_coordinates')
    s = rospy.Service('get_coordinates', get_3D_coordinates, handle_get_3D_coordinates)
    print("Ready")
    rospy.spin()

def main():
    
 
if __name__ == '__main__':
    main()
