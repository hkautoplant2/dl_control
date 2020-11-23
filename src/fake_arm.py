#!/usr/bin/env python

## Description:
# Starts a non anonymous node: fake_arm node and creates one new topic: arm_BP. arm_BP is a boolean check for wether the arm is in position to take an image. If the topic is False, wait  seconds and then publish True to it, otherwise print "still in dnn" 

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import time



def callback_bool(data):
    #rospy.loginfo('image_to_dnn -> arm_BP callback heard:  %s ', data.data)
    pub_BP2 = rospy.Publisher('arm_BP', Bool, queue_size=1)
    print('callback bool in arm ')
    if data.data == True:
        print('still in dnn')
    else:
        print('Arm in base position is false, waiting and then set to true')
        time.sleep(5)
        print('Done waiting, set position to true')
        pub_BP2.publish(True)

def callback(data):
    print('callback real coordinates from dnn')
    pub_BP = rospy.Publisher('arm_BP', Bool, queue_size=1)
    
    pub_BP.publish(False)

def fake_arm():
 
    pub_BP = rospy.Publisher('arm_BP', Bool, queue_size=1)
    bp_sub = rospy.Subscriber("arm_BP", Bool, callback_bool)
    image_sub = rospy.Subscriber('coord',Float32MultiArray,callback)

    rospy.init_node('fake_arm', anonymous=False)

    print('Starting False position testarm')
    pub_BP.publish(False)
    rospy.spin()



if __name__ == '__main__':
    try:
	print("Starting fake node of arm")
        fake_arm()
    except rospy.ROSInterruptException:
        pass
