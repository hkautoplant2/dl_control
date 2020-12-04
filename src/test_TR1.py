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





def main():
    rospy.init_node('master_node', anonymous=False)
    #rate = rospy.Rate(0.015)
    rate = rospy.Rate(0.2)
    inrate = rospy.Rate(2)
    rospy.wait_for_service('go_to_target')
    rospy.wait_for_service('get_pos')

    goto = rospy.ServiceProxy('go_to_target',GoToTarget)
    getpos = rospy.ServiceProxy("get_pos",GetPos)


    #Starting point should be Start
    S = [2200, 0, 1000]
    #Test cases, 3 or 5 of each
    A = [1800, 0, 500]
    B = [2142, -500, 1000]
    C = [1800, -500, 500]

 
    i = 0
    target = B   #Change to B and C 

    while not rospy.is_shutdown():
        print('-----------Starting test---------------')
        if i == 0:
            print('Target is: ', S)
            rospy.wait_for_service('go_to_target')
            response = goto(S[0], S[1], S[2])
            print('Response: ', response.x_current, response.y_current, response.z_current)
            time.sleep(10)
            print('Switch state')
            i = 1
        elif i == 1:
            print('Target is: ', target)
            rospy.wait_for_service('go_to_target')
            response = goto(target[0], target[1], target[2])
            print('Response: ', response.x_current, response.y_current, response.z_current)
            time.sleep(10)
            print('Switch state')
            i = 0
        #current = getpos()
        #print('Current pos: ', current)
        #time.sleep(5)


        print('code done, wait for rate')
        rate.sleep()





    rospy.spin()

if __name__ == '__main__':
    main()
