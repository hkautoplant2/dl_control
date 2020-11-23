#! /usr/bin/env python

## fake master nod to test kinematic server


import rospy
import sys

from dl_control.srv import *



def main():


    x_target = 1800
    y_target= 1
    z_target = 500
    rospy.wait_for_service('go_to_target')
    rospy.wait_for_service('get_pos')
    while not rospy.is_shutdown():
        x_target = input("input coordinates:")
        y_target = input("")
        z_target = input("")

        server1 = rospy.ServiceProxy("get_pos",GetPos)
        print("current_pos=",server1())
        
        server = rospy.ServiceProxy('go_to_target',GoToTarget)
        response = server(x_target,y_target,z_target)

        print("current pos",response)


    #except:
    #    print("could not connect to server")




if __name__=="__main__":

    main()
