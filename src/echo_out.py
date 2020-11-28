#!/usr/bin/env python

## Description:
# Starts a non anonymous node: fake_arm node and creates one new topic: arm_BP. arm_BP is a boolean check for wether the arm is in position to take an image. If the topic is False, wait  seconds and then publish True to it, otherwise print "still in dnn" 

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import time



def callback_bool(data):
    print('Arm is in base position: ', data.data)    


def callback(data):
    print('Received coordinates: ', data.data)

def countdepth_callback(data):
    print('Starting counting depth tries', data)


def fake_arm():
 

    rospy.Subscriber("arm_BP", Bool, callback_bool)
    rospy.Subscriber('coord',Float32MultiArray,callback)
    rospy.Subscriber('depth_counter', Bool, countdepth_callback)

    rospy.init_node('echo_out', anonymous=False)


    rospy.spin()



if __name__ == '__main__':
    try:
	print("Ecko out")
        fake_arm()
    except rospy.ROSInterruptException:
        pass
