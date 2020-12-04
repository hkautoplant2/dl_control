#!/usr/bin/env python


## Description:
# This node get_image subscribes to the topics arm_BP and one of the image topics from ZED. When the arm_BP callback signals True, the image callback goes into the function "do_something".

import roslib
roslib.load_manifest('dl_control')
import rospy
from std_msgs.msg import Float32MultiArray



class Joint_State():
    def __init__(self):
        self.omega = None
        self.alpha = None
        self.beta = None
        self.gamma = None

    def callback_arm(self, data):
        self.alpha = data.data[0]
        self.beta = data.data[1]
        self.gamma = data.data[2]
        self.pub()

    def callback_bm(self, data):
        self.omega = data.data[0]
        self.pub()

    def pub(self):
        if (self.omega and self.alpha and self.beta and self.gamma) != None:
            pub_state = rospy.Publisher('Joint_State_uc', Float32MultiArray, queue_size=1)
            msg = Float32MultiArray()
            msg.data = [self.omega,self.alpha,self.beta,self.gamma]
            pub_state.publish(msg)



def main():
    rospy.init_node('comm_kinematics', anonymous=False)

    jointstate = Joint_State()
    rospy.Subscriber('base_motor_state', Float32MultiArray, jointstate.callback_bm)
    rospy.Subscriber('arm_states', Float32MultiArray, jointstate.callback_arm)

   




    rospy.spin()

if __name__ == '__main__':
    main()
