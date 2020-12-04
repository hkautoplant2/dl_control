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
import math

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from dl_control.srv import*


class Arm:
    def __init__(self):
        self.coord_done = False
        self.coord = None
        self.counter = 0

    def coord_callback(self, data):
        self.coord_done = True
        self.coord = data.data

    def countdepth_callback(self, data):
        self.counter = self.counter + 1
       

    def bp_callback(self, data):
        if data == True:
            print('base position is True')



def transform(current, camera):
    theta = np.arctan(float(current[1])/float(current[0]))
    
    Z = current[2] - camera[0] + 200 #200 mm margin from camera to target on ground
    X = current[0] + camera[1]*np.cos(theta) + camera[2]*np.sin(theta)
    Y = current[1] + camera[1]*np.sin(theta) - camera[2]*np.cos(theta)
    
    return X, Y, Z


def main():
    rospy.init_node('master_node', anonymous=False)
    #rate = rospy.Rate(0.015)
    rate = rospy.Rate(0.1)
    inrate = rospy.Rate(2)
    rospy.wait_for_service('go_to_target')
    rospy.wait_for_service('get_pos')

    goto = rospy.ServiceProxy('go_to_target',GoToTarget)
    getpos = rospy.ServiceProxy("get_pos",GetPos)


    #Starting point should be Start
    Start = [2200, 0, 1100]
    #Test cases, 3 or 5 of each
    A = [1800, 0, 500]
    B = [2142, 500, 1000]
    C = [1800, 500, 500]

 
    
    target = A   #Change to B and C 

    while not rospy.is_shutdown():
        print('-----------Starting test---------------')
        rospy.wait_for_service('go_to_target')
        response = goto(target[0], target[1], target[2])
        print('Response: ', response.x_current, response.y_current, response.z_current)

        
        print('code done, wait for rate')
        rate.sleep()





    rospy.spin()

if __name__ == '__main__':
    main()
