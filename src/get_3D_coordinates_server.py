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

from dl_control.srv import *



def handle_add_two_ints(req):
    #print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    x=10
    y=20
    z=30
    
    return AddTwoIntsResponse(x,y,z)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()
    

def main():
    #get_3D_coordinates_server()
    add_two_ints_server()
    
 
if __name__ == '__main__':
    main()
