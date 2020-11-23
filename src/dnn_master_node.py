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

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from dl_control.srv import *

right_pos = False
dnn = False
retrieve_depth = False
xc_pix = None
yc_pix = None

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
        #t0 = perf_counter()
        p = subprocess.Popen(["bash", "/home/jetson/catkin_ws/src/dl_control/src/shell_inference.sh"])
        (output, err) = p.communicate()
        p.status = p.wait()
        t = time.time() - t0
        print(t, '----------------------------------------------')
       
        found_area, x_c, y_c = read_file()
        
        print found_area, x_c, y_c
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

		        y_middle=(y2-y1)/2 + y1
	                x_middle=(x2-x1)/2 + x1
                        return True, x_middle, y_middle
            else:
                return False, 0.00, 0.00     




def add_two_ints_client(req):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(req)
        return resp1.X, resp1.Y, resp1.Z
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def callback(data):
    global right_pos, retrieve_depth, xc_pix, yc_pix, dnn  
    
    time.sleep(0.5)
    if len(data.data)==3686400 and right_pos == True and dnn == True:
        print('in callback')
        dnn = False
        xc, yc = run_DNN(data)
        xc_pix = xc
        yc_pix = yc
        retrieve_depth = True
        
        print('callback done ', xc, yc)
        right_pos =False
        #time.sleep(1)

def depth_callback(data):
    global right_pos, retrieve_depth, xc_pix, yc_pix 
    pub_BP = rospy.Publisher('arm_BP', Bool, queue_size=1)
    if retrieve_depth == True:
        retrieve_depth = False
        cx = data.width / 2
        cy = data.height / 2
        f = 500 # focal length in pixels HD720 resolution
        b = 0.12 # baseline
        x = int(xc_pix)
        y = int(yc_pix)
      
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='32FC1')
        print(math.isnan(cv_image[y, x]))
        if math.isnan(cv_image[y, x]) == False and math.isinf(cv_image[y, x]) == False:
            Zp = cv_image[y, x]
            Yp = (y - cy) * Zp / f 
            Xp = (x - cx) * Zp / f
            print('point', Xp, Yp, Zp)
        pub_BP.publish(False)
        right_pos =False

def callback_bool(data):
    #rospy.loginfo('image_to_dnn -> arm_BP callback heard:  %s ', data.data)

    global right_pos
    global dnn

    if data.data==True:
      right_pos = True
      dnn = True
    else:
      right_pos = False
    print('callback bool right_pos: ', right_pos)
    return right_pos

def main():
    rospy.init_node('dnn_master_node', anonymous=False)

    directory = '/home/jetson/run_inf_jet'
    os.chdir(directory)

    image_sub = rospy.Subscriber('/zed2/zed_node/left/image_rect_color',Image,callback)
    bp_sub = rospy.Subscriber("arm_BP", Bool, callback_bool)
   
    depth_sub = rospy.Subscriber('/zed2/zed_node/depth/depth_registered',Image, depth_callback)



    '''A = [1.5, 1.5, 1.5]
    B = [1.1, 1.1, 1.1]
    BP = [A, B]
    i = 0
    while i < 10:
        i = i+1
        BP_A = FKIK_service(A)
        if BP_A == True:
            T_A = DNN_service(A)
            A_G = FKIK_service(T_A)
            print('Planting at A successful')
        else:
            print('FKIK BP-A not successful')

        BP_B = FKIK_service(B)
        if BP_B == True:
            T_B = DNN_service(B)
            B_G = FKIK_service(T_B)
            print('Planting at B successful')    
        else:
            print('FKIK BP-B not successful')'''

    rospy.spin()

if __name__ == '__main__':
    main()
