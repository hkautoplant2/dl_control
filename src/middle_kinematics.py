#!/usr/bin/env python


## Description:
# This node get_image subscribes to the topics arm_BP and one of the image topics from ZED. When the arm_BP callback signals True, the image callback goes into the function "do_something".

import roslib
roslib.load_manifest('dl_control')
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool



class Joint_State():
    def __init__(self):
        self.omega = None
        self.alpha = None
        self.beta = None
        self.gamma = None
        self.done = None
        self.done_base = None

    def callback_arm(self, data):
        self.alpha = data.data[1]
        self.beta = data.data[2]
        self.gamma = data.data[3]
        #rospy.loginfo('Arm input data: %s %s %s', data.data[1], data.data[2], data.data[3])
        self.pub()

    def callback_bm(self, data):
        self.omega = data.data[0]
        #rospy.loginfo('BM input: %s', data.data[0])
        self.pub()

    def callback_done(self, data):
        self.done = data.data

    def callback_done_base(self, data):
        self.done_base = data.data

    def pub(self):
        print(self.omega, self.alpha, self.beta, self.gamma, self.done, self.done_base)
        if (self.done==False or self.done_base == False) and (self.omega != None and self.alpha != None and self.beta != None and self.gamma != None): 
            print('Publishing joint state', self.omega, self.alpha, self.beta, self.gamma, self.done)
            pub_state = rospy.Publisher('Joint_State_uc', Float32MultiArray, queue_size=1)
            msg = Float32MultiArray()
            msg.data = [self.omega, self.alpha, self.beta, self.gamma]
            pub_state.publish(msg)



def main():
    rospy.init_node('middle_kinematics', anonymous=False)

    jointstate = Joint_State()
    rospy.Subscriber('base_motor_state', Float32MultiArray, jointstate.callback_bm)
    rospy.Subscriber('arm_states', Float32MultiArray, jointstate.callback_arm)
    rospy.Subscriber('finished', Bool, jointstate.callback_done)
    rospy.Subscriber('finished_base', Bool, jointstate.callback_done_base)
   




    rospy.spin()

if __name__ == '__main__':
    main()
