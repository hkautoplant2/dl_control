#!/usr/bin/env python

## Description:
# Starts a non anonymous node: fake_arm node and creates two new topics: coord_3D and arm_BP. coord_3D acts as the topic for the relative 3D coordinates (XYZ) to a pixel pair in the image. arm_BP is a boolean check for wether the arm is in position to take an image. 

import rospy
import cv2
import datetime
import time
import subprocess
import os
import fnmatch
import glob
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


right_pos = False

def show(pic_data):
        p2 = subprocess.Popen(["bash", "/home/jetson/catkin_ws/src/dl_control/src/shell_remove.sh"])
        (output2, err2) = p2.communicate()
        p2.status = p2.wait()
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(pic_data, "bgr8")
        basename = "log_image"
        suffix = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
        filename = "_".join([basename, suffix])
        filename = filename + '.png'
        path = os.getcwd() + '/' + filename
        cv2.imwrite(filename, cv_image)
        t0=time.time()
        #t0 = perf_counter()
        p = subprocess.Popen(["bash", "/home/jetson/catkin_ws/src/dl_control/src/shell_inference.sh"])
        (output, err) = p.communicate()
        p.status = p.wait()
        t = time.time() - t0
        print(t, '----------------------------------------------')
       
        found_area, x_c, y_c = read_file()
        
        print found_area, x_c, y_c
        time.sleep(10)

def read_file():
#Program that search in each annotators file, after good area. Then it count the midpoint in the bounding box 
    txt_file = glob.glob('/home/jetson/res_inf_jet/*.txt')
    
    with open(txt_file[0], "r") as f:  #The annotators file for the give image 
        if os.stat(txt_file[0]).st_size == 0:
            print('empty')
            return False, 0.00, 0.00
        else:
	    for line in f:
                entry = line.split()
                print(type(line))
                if entry:
                    if entry[0]=="stone":
		        x1=float(entry[4])
	                y1=float(entry[5])
		        x2=float(entry[6])
		        y2=float(entry[7])

		        y_middle=(y2-y1)/2 + y1
	                x_middle=(x2-x1)/2 + x1
                        return True, x_middle, y_middle
            else:
                return False, 0.00, 0.00                
 
def callback_bool(data):
    #rospy.loginfo('image_to_dnn -> arm_BP callback heard:  %s ', data.data)

    global right_pos

    if data.data==True:
      right_pos = True
    else:
      right_pos = False
    return right_pos


def callback(data):
    if len(data.data)==3686400 and right_pos == True:
        show(data)
 
def main():
    rospy.init_node('shell_test', anonymous=False)
    directory = '/home/jetson/run_inf_jet'
    os.chdir(directory)

    rospy.loginfo('----------test shell script-----------')
    image_sub = rospy.Subscriber('/zed2/zed_node/left/image_rect_color',Image,callback)
    bp_sub = rospy.Subscriber("arm_BP", Bool, callback_bool)
      
    rospy.spin()

if __name__ == '__main__':
    try:
	print("Starting test of shell")
        main()
    except rospy.ROSInterruptException:
        pass
