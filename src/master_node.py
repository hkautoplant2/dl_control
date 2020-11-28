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
    Z = current[2] - camera[1]*1000 + 400
    print('Old transform: ', X, Y, Z)

    alpha = np.arctan(current[0]/current[1])
    if current[1] >= 0:
        theta = alpha + 1.5708
    else:
        theta = alpha + 2.3562
    print('Theta is: ', theta, 'alpha is: ', alpha)
    PX = current[0] + camera[0]*1000*np.cos(theta) + camera[2]*1000*np.sin(theta)
    PY = current[1] - camera[0]*1000*np.sin(theta) + camera[2]*1000*np.cos(theta)
    PZ = Z
    print('New transform: ', PX, PY, PZ)
    
    return PX, PY, PZ


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
    
    waiting = False
    i = 0
    #TODO Input guard for taking long time to find non NaN depth image
    while not rospy.is_shutdown():
        print('Start of while loop')
        if i == 0:
            if not waiting:
                print('State 0 = A, moving to BP A')
                rospy.wait_for_service('go_to_target')
                resp = goto(A[0], A[1], A[2])
                response = resp
                print('publish base A position True, response: ', response)
                pub_BP.publish(True)
                waiting = True
            while not rospy.is_shutdown() and i == 0 and waiting:
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
                        waiting = False
                        break
                    else: 
                        print('Target area was out of bound, moving to next state')
                        i = 1 
                        waiting = False
                        break
                elif arm.counter > 15:
                    print('Depth not found, moving to next step')
                    i = 1
                    pub_BP.publish(False)
                    arm.counter = 0
                    waiting = False
                    break 
                   
                inrate.sleep()
                    

        
        if i == 1:
            if not waiting:
                print('State 1 = B, moving to BP B')
                rospy.wait_for_service('go_to_target')
                resp = goto(B[0], B[1], B[2])
                response = resp
                print('publish base B position True, response: ', response)
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
                        time.sleep(2)
                        i = 0 
                        waiting = False
                        break
                    else: 
                        print('Target area was out of bound, moving to next state')
                        i = 0 
                        waiting = False
                        break
                elif arm.counter > 15:
                    print('Depth not found, noving to next step')
                    i = 1
                    pub_BP.publish(False)
                    waiting = False
                    arm.counter = 0
                    break
                    
                inrate.sleep()
              
        
        print('code done, wait for rate')
        rate.sleep()





    rospy.spin()

if __name__ == '__main__':
    main()
