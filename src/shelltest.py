#!/usr/bin/env python

## Description:
# Starts a non anonymous node: fake_arm node and creates two new topics: coord_3D and arm_BP. coord_3D acts as the topic for the relative 3D coordinates (XYZ) to a pixel pair in the image. arm_BP is a boolean check for wether the arm is in position to take an image. 

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import time
import subprocess


def callback(data):
    #rospy.loginfo('fake_arm_ctrl -> coord_3D callback heard coordinates: %s ', data.data)
    x=1


    

def main():
    rospy.init_node('shell_test', anonymous=False)



    rospy.loginfo('----------test shell script-----------')
   

    while not rospy.is_shutdown():
        
        
        print('Testing')
        subprocess.Popen(["bash", "aaashell.sh"])
        time.sleep(60)

#process = subprocess.Popen(['bash', '/aaashell.sh'],
#                     stdout=subprocess.PIPE, 
#                     stderr=subprocess.PIPE)

#stdout, stderr = process.communicate()
        
   
   #rospy.spin()


if __name__ == '__main__':
    try:
	print("Starting test of shell")
        main()
    except rospy.ROSInterruptException:
        pass
