#!/usr/bin/env python

## Description:
# Starts a non anonymous node: fake_arm node and creates one new topic: arm_BP. arm_BP is a boolean check for wether the arm is in position to take an image. If the topic is False, wait  seconds and then publish True to it, otherwise print "still in dnn" 

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import time



def callback_bool(data):
    print('Arm is in base position: ', data.data)    
    pub_coord = rospy.Publisher('coord', Float32MultiArray, queue_size=1)
    if data.data == True:
        print('Base position True, getting coordinates')
        
        time.sleep(2)
        Pxc = 2200
        Pyc = -200
        Pzc = 700
        coord_fma = Float32MultiArray(data=[Pxc, Pyc, Pzc])
        pub_coord.publish(coord_fma)





def fake_arm():
 

    rospy.Subscriber("arm_BP", Bool, callback_bool)

    rospy.init_node('fake_dnn', anonymous=False)


    rospy.spin()



if __name__ == '__main__':
    try:
	print("fake_dnn")
        fake_arm()
    except rospy.ROSInterruptException:
        pass
