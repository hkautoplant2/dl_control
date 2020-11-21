#!/usr/bin/env python

## Description:
# Starts a non anonymous node: fake_arm node and creates two new topics: coord_3D and arm_BP. coord_3D acts as the topic for the relative 3D coordinates (XYZ) to a pixel pair in the image. arm_BP is a boolean check for wether the arm is in position to take an image. 

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import time
import subprocess
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import datetime

right_pos = False

def callback(data):
    #rospy.loginfo('fake_arm_ctrl -> coord_3D callback heard coordinates: %s ', data.data)
    x=1

def show(pic_data):


        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(pic_data, "bgr8")
        #directory = r'C:\Users\Rajnish\Desktop\GeeksforGeeks'
        #os.chdir(directory)
        basename = "mylogfile"
        suffix = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
        filename = "_".join([basename, suffix]) # e.g. 'mylogfile_120508_171442'
        filename = filename + '.png'
        path = os.getcwd() + '/' + filename
        #print(filename)

        print('save image')
        cv2.imwrite(filename, cv_image)
        print('image saved')
        subprocess.Popen(["bash", "/home/jetson/catkin_ws/src/dl_control/src/aaashell.sh"])
        time.sleep(60)
        print('done sleeping')

        #img = cv2.imread(path)
        #cv2.imshow("Image window", img)
        
       
        #cv2.waitKey(1)

def callback_bool(data):
    #rospy.loginfo('image_to_dnn -> arm_BP callback heard:  %s ', data.data)


    global right_pos

    if data.data==True:
      right_pos = True
    else:
      right_pos = False
    return right_pos



def callback2(data):
    #print(data.header)
    
    
    

    if len(data.data)==3686400 and right_pos == True:
        print('if')
        #print(data.header)
        show(data)
        print('image done')
        
        #time.sleep(2)

        '''bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        #directory = r'C:\Users\Rajnish\Desktop\GeeksforGeeks'
        #os.chdir(directory)
        filename = 'image.png'
        path = os.getcwd() + '/' + filename

        cv2.imwrite(filename, cv_image)
        img = cv2.imread(path)
        cv2.imshow("Image window", img)
        #cv2.waitKey(1)
        #time.sleep(2)'''
        
            
    
    

def main():
    rospy.init_node('shell_test', anonymous=False)
    directory = '/home/jetson/run_inf_jet'
    os.chdir(directory)



    rospy.loginfo('----------test shell script-----------')
    image_sub = rospy.Subscriber('/zed2/zed_node/left/image_rect_color',Image,callback2)
    bp_sub = rospy.Subscriber("arm_BP", Bool, callback_bool)
   

    #while not rospy.is_shutdown():
        
        
        #print('Testing')
        #time.sleep(2)
        #print('yo')
        #print(os.getcwd())
        #callback2()
        



        #subprocess.Popen(["bash", "aaashell.sh"])
        

#process = subprocess.Popen(['bash', '/aaashell.sh'],
#                     stdout=subprocess.PIPE, 
#                     stderr=subprocess.PIPE)

#stdout, stderr = process.communicate()
        
   
    rospy.spin()


if __name__ == '__main__':
    try:
	print("Starting test of shell")
        main()
    except rospy.ROSInterruptException:
        pass
