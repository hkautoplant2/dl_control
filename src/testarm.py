#!/usr/bin/env python

## Description:
# Starts a non anonymous node: fake_arm node and creates two new topics: coord_3D and arm_BP. coord_3D acts as the topic for the relative 3D coordinates (XYZ) to a pixel pair in the image. arm_BP is a boolean check for wether the arm is in position to take an image. 

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import time

def callback(data):
    rospy.loginfo('fake_arm_ctrl -> coord_3D callback heard coordinates: %s ', data.data)


def fake_arm():
    sub_coord = rospy.Subscriber('coord_3D', Float32MultiArray, callback)
    pub_BP = rospy.Publisher('arm_BP', Bool, queue_size=10)
    ## TODO Change coord_3D to take an array instead of single float
    rospy.init_node('fake_arm_ctrl', anonymous=False)
    rate = rospy.Rate(3) # Hz
    i= 0
    
    while not rospy.is_shutdown():
        
	if i == 18:
	   rospy.loginfo('right position')
	   pub_BP.publish(True)
           time.sleep(3)
	   rate.sleep()
           i=0
        else:  
           #hello_str = "hello world %s" % rospy.get_time()
           rospy.loginfo('moving')
           #pub_coord.publish(67)
	   pub_BP.publish(False)
           rate.sleep()
        
        i=i+1



if __name__ == '__main__':
    try:
	print("Starting fake node of arm")
        fake_arm()
    except rospy.ROSInterruptException:
        pass
