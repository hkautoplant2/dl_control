#!/usr/bin/env python

## Description:
# Starts a non anonymous node: fake_arm node and creates two new topics: coord_3D and arm_BP. coord_3D acts as the topic for the relative 3D coordinates (XYZ) to a pixel pair in the image. arm_BP is a boolean check for wether the arm is in position to take an image. 

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import time

class FakeArm:
    def __init__(self):
        self.dnn_done = False
        self.is_in_BP = None
        self.relative_coord = None
        
    def coordinate_callback(self, msg):
        rospy.loginfo('fake_arm_ctrl -> coord_3D callback heard coordinates: %s ', msg.data)
        self.relative_coord = msg.data

    def visual_callback(self, msg):
        rospy.loginfo('fake_arm_ctrl -> visual calculations done: %s ', msg.data)
        self.dnn_done = msg.data

def callback(data):
    #rospy.loginfo('fake_arm_ctrl -> coord_3D callback heard coordinates: %s ', data.data)
    x=1

def callback_vis(data):
    rospy.loginfo('fake_arm_ctrl -> visual_done: Neural network is done: %s ', data.data)
    time.sleep(1)
    

def fake_arm():
    rospy.init_node('fake_arm_ctrl', anonymous=False)

    fakearm = FakeArm()

    rospy.Subscriber('/coord_3D', Float32MultiArray, fakearm.coordinate_callback)
    rospy.Subscriber('/visual_done', Bool, fakearm.visual_callback)

    pub_BP = rospy.Publisher('/arm_BP', Bool, queue_size=1)
    pub_vis = rospy.Publisher('/visual_done', Bool, queue_size=1)

    pub_BP.publish(False)
    rospy.loginfo('----------fake_arm_ctrl -> time control first False-----------')
    time.sleep(10)

    while not rospy.is_shutdown():
        
        
        pub_BP.publish(True)
        #time.sleep(10)
        if fakearm.dnn_done == False:
            rospy.loginfo('fake_arm_ctrl -> Calculations done, reset settings')
            pub_vis.publish(False)
            pub_BP.publish(False)
            rospy.loginfo('fake_arm_ctrl -> MOVING ARM')
            time.sleep(10)
    
   
   #rospy.spin()


if __name__ == '__main__':
    try:
	print("Starting fake node of arm")
        fake_arm()
    except rospy.ROSInterruptException:
        pass
