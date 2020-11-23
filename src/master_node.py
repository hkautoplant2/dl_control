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


def main():
    rospy.init_node('master_node', anonymous=False)
    rospy.wait_for_service('go_to_target')
    rospy.wait_for_service('get_pos')

    A = [1.5, 1.5, 1.5]
    B = [1.1, 1.1, 1.1]

    i = 0

    if i == 0:
        getpos1 = rospy.ServiceProxy("get_pos",GetPos)
        current_pos = getpos1()
        if current_pos == A:
            arm_BP.publish(True)
        else: 
            goto = rospy.ServiceProxy('go_to_target',GoToTarget)
            response = goto(A[0], A[1], A[2])
            arm_BP.publish(True)






    rospy.spin()

if __name__ == '__main__':
    main()
