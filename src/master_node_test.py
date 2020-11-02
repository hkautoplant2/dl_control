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


def add_two_ints_client(req):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(req)
        return resp1.X, resp1.Y, resp1.Z
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



def main():
    #print(DNN_service())
    
    rospy.init_node('master_node', anonymous=False)
    print(add_two_ints_client(True))
    '''A = [1.5, 1.5, 1.5]
    B = [1.1, 1.1, 1.1]
    BP = [A, B]
    i = 0
    while i < 10:
        i = i+1
        BP_A = FKIK_service(A)
        if BP_A == True:
            T_A = DNN_service(A)
            A_G = FKIK_service(T_A)
            print('Planting at A successful')
        else:
            print('FKIK BP-A not successful')

        BP_B = FKIK_service(B)
        if BP_B == True:
            T_B = DNN_service(B)
            B_G = FKIK_service(T_B)
            print('Planting at B successful')    
        else:
            print('FKIK BP-B not successful')'''

    rospy.spin()

if __name__ == '__main__':
    main()
