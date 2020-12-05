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

    rate = rospy.Rate(2)

    arm = Arm()

    rospy.wait_for_service('go_to_target')
    rospy.wait_for_service('get_pos')
    goto = rospy.ServiceProxy('go_to_target',GoToTarget)
    getpos = rospy.ServiceProxy("get_pos",GetPos)

    pub_BP = rospy.Publisher('arm_BP', Bool, queue_size=1)

    rospy.Subscriber('/coord',Float32MultiArray,arm.coord_callback)
    rospy.Subscriber('/arm_BP', Bool, arm.bp_callback)
    rospy.Subscriber('/dnn_break', Bool, arm.break_callback)

    

    A = [2200, 150, 900]
    B = [2000, -700, 1000]
    C = [2200, 0, 1200] 

    base_poses = [A, B, C]
    states = ['A', 'B', 'C']
 
    print('Starting sequence, publish False')
    time.sleep(1)
    pub_BP.publish(False)
    waiting = False

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
                    time.sleep(3)
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
        print('code done, wait for rate', waiting)
        rate.sleep()
    
    rospy.spin()



if __name__ == '__main__':
    main()
