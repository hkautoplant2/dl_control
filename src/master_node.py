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

class Arm:
    def __init__(self):
        self.coord_done = False
        self.coord = None

    def coord_callback(self, data):
        print('coord callback: ', data)
        self.coord_done = True
        self.coord = data.data

    def bp_callback(self, data):
        print('base position callback: ', data)
        if data == True:
            print('base position is True')
        else:
            print('base position is False')

def main():
    rospy.init_node('master_node', anonymous=False)
    rate = rospy.Rate(0.5)
    rospy.wait_for_service('go_to_target')
    rospy.wait_for_service('get_pos')

    goto = rospy.ServiceProxy('go_to_target',GoToTarget)
    getpos = rospy.ServiceProxy("get_pos",GetPos)

    pub_BP = rospy.Publisher('arm_BP', Bool, queue_size=1)

    arm = Arm()

    rospy.Subscriber('/coord',Float32MultiArray,arm.coord_callback)
    rospy.Subscriber('/arm_BP', Bool, arm.bp_callback)

    A = [2500, 1, 1000]
    B = [2500, 1, 1000]

 
    pub_BP.publish(False)
    print('Starting sequence, publish False')
    time.sleep(3)
    

    '''
    print('publish True')
    pub_BP.publish(True)
    while not rospy.is_shutdown():
        print('arm.coord_done', arm.coord_done)
        if arm.coord_done == True:
            arm.coord_done = False
            break
        rate.sleep()
    print('Out of first plant')
    print('Retrieved coordinates: ', arm.coord[0], arm.coord[1], arm.coord[2])
    time.sleep(2)
    print('Reset')
    time.sleep(2)'''

    i = 0
    '''
    while not rospy.is_shutdown():
        if i == 0:
            response = goto(A[0], A[1], A[2])
            print('publish True')
            pub_BP.publish(True)
            if arm.coord_done == True:
                arm.coord_done = False
                print('Coordinates for spot received, moving there')
                response = goto(arm.coord[0], arm.coord[1], arm.coord[2])
                i = 1
        if i == 1:
            response = goto(B[0], B[1], B[2])
            print('publish True')
            pub_BP.publish(True)
            if arm.coord_done == True:
                arm.coord_done = False
                print('Coordinates for spot received, moving there')
                response = goto(arm.coord[0], arm.coord[1], arm.coord[2])
                i = 0
        rate.sleep()'''





    rospy.spin()

if __name__ == '__main__':
    main()
