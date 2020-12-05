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


    rate = rospy.Rate(0.1)

    A = [1900, 150, 1000]
    B = [1800, -700, 1000]
    C = [2200, 0, 1200] 
    base_poses = [A, B, C]
    states = ['A', 'B', 'C']

    while not rospy.is_shutdown():
        print('*****************Start of plantation loop*********************')
        for state, BP in enumerate(base_poses):
            if not rospy.is_shutdown():
                
                print 'State', states[state], ':', BP, '---------------------'




    rospy.spin()

if __name__ == '__main__':
    main()
