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
        self.dnn_break = False

    def coord_callback(self, data):
        self.coord_done = True
        self.coord = data.data

    def break_callback(self, data):
        self.dnn_break = data.data
       

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
    rate = rospy.Rate(2)
    inrate = rospy.Rate(2)
    rospy.wait_for_service('go_to_target')
    rospy.wait_for_service('get_pos')

    goto = rospy.ServiceProxy('go_to_target',GoToTarget)
    getpos = rospy.ServiceProxy("get_pos",GetPos)

    pub_BP = rospy.Publisher('arm_BP', Bool, queue_size=1)

    arm = Arm()

    rospy.Subscriber('/coord',Float32MultiArray,arm.coord_callback)
    rospy.Subscriber('/arm_BP', Bool, arm.bp_callback)
    rospy.Subscriber('/dnn_break', Bool, arm.break_callback)

    A = [1900, 150, 1000]
    B = [1800, -700, 1000]

 
    pub_BP.publish(False)
    print('Starting sequence, publish False')
    time.sleep(3)
    
    waiting = False
    i = 0

    while not rospy.is_shutdown():
        print('-----------Start of while loop---------------')
        if i == 0:
            if not waiting:
                print('State 0 = A, moving to BP A')
                rospy.wait_for_service('go_to_target')
                resp = goto(A[0], A[1], A[2])
                response = resp
                print('publish base A position True, response: ', response.x_current, response.y_current, response.z_current)
                time.sleep(2)
                pub_BP.publish(True)
                waiting = True
            while not rospy.is_shutdown() and i == 0 and waiting:
                if arm.coord_done == True:
                    arm.coord_done = False
                    pub_BP.publish(False)
                    print( "Coordinates from camera: ", arm.coord )
                    XA, YA, ZA = transform([response.x_current, response.y_current, response.z_current], arm.coord)
                    #XA, YA, ZA = transform([2200, -200, 1100], arm.coord)
                    print("Target/spot coordinates for end-effector: ", XA, YA, ZA )
                    if (1500 <= XA <= 3000) and (ZA >= 220):
                        rospy.wait_for_service('go_to_target')
                        response = goto(XA, YA, ZA)
                        print('Reached target spot, performing plantation...')
                        time.sleep(7)
                        print('Plantation done, moving back to A')
                        rospy.wait_for_service('go_to_target')
                        response = goto(A[0], A[1], A[2])
                        i = 1 
                        waiting = False
                        break
                    else: 
                        print('Target area was out of bound, staying in A')
                        i = 1 
                        pub_BP.publish(False)
                        arm.dnn_break = False
                        waiting = False
                        arm.coord_done = False
                        time.sleep(1)
                        break
                elif arm.dnn_break:
                    print('Depth not found, staying A')
                    i = 1
                    pub_BP.publish(False)
                    arm.dnn_break = False
                    waiting = False
                    arm.coord_done = False
                    time.sleep(1)
                    break 
                   
                rate.sleep()
                    

        
        elif i == 1:
            if not waiting:
                print('State 1 = B, moving to BP B')
                rospy.wait_for_service('go_to_target')
                resp = goto(B[0], B[1], B[2])
                response = resp
                print('publish base B position True, response: ', response.x_current, response.y_current, response.z_current)
                time.sleep(2)
                pub_BP.publish(True)
                waiting = True
            while not rospy.is_shutdown() and i == 1 and waiting:
                if arm.coord_done == True:
                    arm.coord_done = False
                    pub_BP.publish(False)
                    print( "Coordinates from camera: ", arm.coord )
                    XB, YB, ZB = transform([response.x_current, response.y_current, response.z_current], arm.coord)
                    print("Target/spot coordinates for end-effector: ", XB, YB, ZB )
                    if 1500 <= XB <= 3000 and (ZB >= 220):
                        rospy.wait_for_service('go_to_target')
                        response = goto(XB, YB, ZB)
                        print('Reached target spot, performing plantation...')
                        time.sleep(7)
                        print('Plantation done, moving back to B')
                        rospy.wait_for_service('go_to_target')
                        response = goto(B[0], B[1], B[2])
                        i = 0 
                        waiting = False
                        break
                    else: 
                        print('Target area was out of bound, staying in B')
                        i = 0 
                        waiting = False
                        pub_BP.publish(False)
                        arm.dnn_break = False
                        waiting = False
                        arm.coord_done = False
                        time.sleep(1)
                        break
                elif arm.dnn_break:
                    print('Depth not found, staying in B')
                    i = 0
                    pub_BP.publish(False)
                    waiting = False
                    arm.dnn_break = False
                    arm.coord_done = False
                    time.sleep(1)
                    break
                    
                rate.sleep()
              
        
        print('code done, wait for rate', i, waiting)
        rate.sleep()





    rospy.spin()

if __name__ == '__main__':
    main()
