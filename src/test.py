#!/usr/bin/env python


## Description:
# This node acts as the master node for controlling the deep neural network part of the system. It is controlled by the topic \arm_BP which when True starts the network on the host computer, performs inference, calculates the center pixels of a good area, and retrieves the real life coordinates for that pixel pair. 

import roslib
roslib.load_manifest('dl_control')
import rospy
import cv2
import time
import os
import numpy as np
import datetime
import subprocess
import glob
import math


def main():
    rospy.init_node('test_node', anonymous=False)


    rate = rospy.Rate(2)

    A = [1900, 150, 1000]
    B = [1800, -700, 1000]
    C = [2200, 0, 1200] 
    base_poses = [A, B, C]
    states = ['A', 'B', 'C']

    while not rospy.is_shutdown():
        print('*****************Start of plantation loop*********************')
        for state, BP in enumerate(base_poses):
            if not rospy.is_shutdown():
                if not waiting:
                    print 'State', states[state], ':', BP, '---------------------'
                    rospy.wait_for_service('go_to_target')
                    resp = goto(BP[0], BP[1], BP[2])
                    response = resp
                    print 'Current position: ', response.x_current, response.y_current, response.z_current
                    time.sleep(2)
                    pub_BP.publish(True)
                    waiting = True
                
                while not rospy.is_shutdown() and waiting:
                    if arm.coord_done == True:
                        arm.coord_done = False
                        pub_BP.publish(False)
                        print( "Coordinates from camera: ", arm.coord )
                        Xnew, Ynew, Znew = transform([response.x_current, response.y_current, response.z_current], arm.coord)
                        #XA, YA, ZA = transform([2200, -200, 1100], arm.coord)
                        print("Target/spot coordinates for end-effector: ", Xnew, Ynew, Znew )
                        if (1500 <= Xnew <= 3000) and (Znew >= 220):
                            rospy.wait_for_service('go_to_target')
                            response = goto(Xnew, Ynew, Znew)
                            print('Reached target spot, performing plantation...')
                            time.sleep(7)
                            print('Plantation done, moving back to BP')
                            rospy.wait_for_service('go_to_target')
                            response = goto(BP[0], BP[1], BP[2])
                            waiting = False
                            break
                        else: 
                            print('Target area was out of bound, staying in position')
                            pub_BP.publish(False)
                            arm.dnn_break = False
                            waiting = False
                            arm.coord_done = False
                            time.sleep(1)
                            break
                    elif arm.dnn_break:
                        print('Depth not found, staying in position')
                        pub_BP.publish(False)
                        arm.dnn_break = False
                        waiting = False
                        arm.coord_done = False
                        time.sleep(1)
                        break 
                    
                    rate.sleep()
        print('code done, wait for rate', i, waiting)
        rate.sleep()
    
    rospy.spin()

if __name__ == '__main__':
    main()
