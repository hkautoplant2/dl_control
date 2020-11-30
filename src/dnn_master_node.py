#!/usr/bin/env python


## Description:
# This node acts as the master node for controlling the deep neural network part of the system. It is controlled by the topic \arm_BP which when True starts the network on the host computer, performs inference, calculates the center pixels of a good area, and retrieves the real life coordinates for that pixel pair. 

import roslib
roslib.load_manifest('dl_control')
import rospy
import cv2
import time
import os
import numpy as np
import datetime
import subprocess
import glob
import math

from sensor_msgs.msg import Imu
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

from dl_control.srv import*

right_pos = False
retrieve_depth = False
xc_pix = None
yc_pix = None
depth_done = False
X = 0
Y = 0
Z = 0
counter = 0

def run_DNN(pic_data):
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
        p = subprocess.Popen(["bash", "/home/jetson/catkin_ws/src/dl_control/src/shell_inference.sh"])
        (output, err) = p.communicate()
        p.status = p.wait()
        t = time.time() - t0
        print(t, 'Ran DNN')
       
        found_area, x_c, y_c = read_file()
        
        #time.sleep(10)
        return x_c, y_c

def read_file():
#Program that search in each annotators file, after good area. Then it count the midpoint in the bounding box 
    txt_file = glob.glob('/home/jetson/res_inf_jet/*.txt')
    
    with open(txt_file[0], "r") as f:  #The annotators file for the given image 
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
                        print('Stone coordinates: ', x1, y1, x2, y2)
		        y_middle=(y2+y1)/2 
	                x_middle=(x2+x1)/2 
                        return True, int(x_middle), int(y_middle)
            else:
                return False, 0.00, 0.00      #x (1280), y (720)



def callback(data):
    global right_pos, retrieve_depth, xc_pix, yc_pix, depth_done  
    time.sleep(0.5)
    if len(data.data)==3686400 and right_pos == True:
        print('in callback')
        right_pos =False
        xc, yc = run_DNN(data)
        xc_pix = int(xc)
        yc_pix = int(yc)
        retrieve_depth = True   #Let the depth image callback know we have pixels to calculate on
        
        print('callback done, target pixels:  ', xc, yc)
        
        #time.sleep(1)

def depth_callback(data):
    global right_pos, retrieve_depth, xc_pix, yc_pix, depth_done, X, Y, Z 
    
    pub_coord = rospy.Publisher('coord', Float32MultiArray, queue_size=1)
    pub_counter = rospy.Publisher('depth_counter', Bool, queue_size=1)

    if retrieve_depth == True:
        xci = data.width / 2
        yci = data.height / 2
        f = 500 # focal length in pixels HD720 resolution
        b = 0.12 # baseline
        Pxi = int(xc_pix)
        Pyi = int(yc_pix)
      
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='32FC1')
        print('Is Nan: ', math.isnan(cv_image[Pyi, Pxi]), math.isinf(cv_image[Pyi, Pxi]), 'retrieve depth: ', retrieve_depth)

        R = cv_image[Pyi, Pxi]*1000 #Scale to millimeters
        if math.isnan(cv_image[Pyi, Pxi]) == False and math.isinf(cv_image[Pyi, Pxi]) == False and retrieve_depth:
            retrieve_depth = False
            print('Pxi: ', Pxi, 'Pyi: ', Pyi)
            Pypc = xci-Pxi 
            Pzpc = yci-Pyi
            Pyc = R*Pypc/f
            Pzc = R*Pzpc/f
            Pxc = np.sqrt(R**2 - Pyc**2 - Pzc**2)

            print('R', R, 'cv image', cv_image[Pyi, Pxi])
            print('Camera, depth= ', Pxc, 'left for camera= ', Pyc, 'up for camera= ', Pzc)

            coord_fma = Float32MultiArray(data=[Pxc, Pyc, Pzc])
            pub_coord.publish(coord_fma)
            depth_done = True
        elif retrieve_depth:
            retrieve_depth = True
            pub_counter.publish(True)
            depth_done = True
            
        

def callback_bool(data):
    #rospy.loginfo('image_to_dnn -> arm_BP callback heard:  %s ', data.data)

    global right_pos

    if data.data==True:
      right_pos = True
    else:
      right_pos = False
    print('callback bool right_pos: ', right_pos)
    #return right_pos

def main():
    rospy.init_node('dnn_master_node', anonymous=False)

    directory = '/home/jetson/run_inf_jet'
    os.chdir(directory)

    image_sub = rospy.Subscriber('/zed2/zed_node/left/image_rect_color',Image,callback)
    bp_sub = rospy.Subscriber("arm_BP", Bool, callback_bool)
    depth_sub = rospy.Subscriber('/zed2/zed_node/depth/depth_registered',Image, depth_callback)



    rospy.spin()

if __name__ == '__main__':
    main()
