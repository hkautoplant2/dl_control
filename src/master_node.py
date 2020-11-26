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
    X = current[0] + camera[1]*1000
    Y = 1 #TODO current[1] + camera[0]*1000
    Z = current[2] - camera[2]*1000 + 200
    return X, Y, Z


def main():
    rospy.init_node('master_node', anonymous=False)
    rate = rospy.Rate(0.015)
    inrate = rospy.Rate(0.5)
    rospy.wait_for_service('go_to_target')
    rospy.wait_for_service('get_pos')

    goto = rospy.ServiceProxy('go_to_target',GoToTarget)
    getpos = rospy.ServiceProxy("get_pos",GetPos)

    pub_BP = rospy.Publisher('arm_BP', Bool, queue_size=1)

    arm = Arm()

    rospy.Subscriber('/coord',Float32MultiArray,arm.coord_callback)
    rospy.Subscriber('/arm_BP', Bool, arm.bp_callback)
    rospy.Subscriber('/depth_counter', Bool, arm.countdepth_callback)

    A = [2200, 1, 1100]
    B = [2000, 1, 1000]

 
    pub_BP.publish(False)
    print('Starting sequence, publish False')
    time.sleep(3)
    

    i = 0
    #TODO Input guard for taking long time to find non NaN depth image
    while not rospy.is_shutdown():
        print('Start of while loop')
        if i == 0:
            print('State 0 = A, moving to BP A')
            rospy.wait_for_service('go_to_target')
            print('servic ready, go to A')
            response = goto(A[0], A[1], A[2])
            print('publish base A position True, response: ', response.x_current)
            pub_BP.publish(True)
            while not rospy.is_shutdown() and i == 0:
                if arm.coord_done == True:
                    arm.coord_done = False
                    pub_BP.publish(False)
                    print( "Coordinates from camera: ", arm.coord )
                    XA, YA, ZA = transform([response.x_current, response.y_current, response.z_current], arm.coord)
                    print("Target/spot coordinates for end-effector: ", XA, YA, ZA )
                    if (1500 <= XA <= 3000) and (ZA >= 220):
                        rospy.wait_for_service('go_to_target')
                        response = goto(XA, YA, ZA)
                        print('Reached target spot, performing plantation...')
                        time.sleep(2)
                        i = 1 
                        break
                    else: 
                        print('Target area was out of bound, moving to next state')
                        i = 1 
                        break
                elif arm.counter > 15:
                    print('Depth not found, noving to next step')
                    i = 1
                    break 
                   
                inrate.sleep()
                    

        
        if i == 1:
            print('State 1 = B, moving to BP B')
            rospy.wait_for_service('go_to_target')
            print('servic ready, go to B')
            response = goto(B[0], B[1], B[2])
            print('publish base B position True, response: ', response.x_current)
            pub_BP.publish(True)
            while not rospy.is_shutdown() and i == 1:
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
                        time.sleep(2)
                        i = 0 
                        break
                    else: 
                        print('Target area was out of bound, moving to next state')
                        i = 0 
                        break
                elif arm.counter > 15:
                    print('Depth not found, noving to next step')
                    i = 1
                    break
                    
                inrate.sleep()
              
        
        print('code done, wait for rate')
        rate.sleep()





    rospy.spin()

if __name__ == '__main__':
    main()
