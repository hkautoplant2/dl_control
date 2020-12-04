#!/usr/bin/env python


## Description:
# This node get_image subscribes to the topics arm_BP and one of the image topics from ZED. When the arm_BP callback signals True, the image callback goes into the function "do_something".

import roslib
roslib.load_manifest('dl_control')
import rospy
import cv2
import time
import datetime
import os
import numpy as np
import math

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from dl_control.srv import*

right_pos = False
retrieve_depth = False
xc_pix = None
yc_pix = None
depth_done = False
Xa = 0
Ya = 0
Za = 0
depth_counter = 0


a = [640, 360]
b = [640-100, 360-100]
c = [640-500, 360+100]
d = [640+500, 360-100]
e = [640+100, 360+100]



def callback(data):
    global right_pos, retrieve_depth, xc_pix, yc_pix, depth_done, a, b, c, d, e 
    time.sleep(0.5)
    if len(data.data)==3686400 and right_pos == True:
        print('in callback')
        right_pos =False
        #TODO Add to mark out the selected pixels on saved image
        #Save image
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(pic_data, "bgr8")
        cv2.circle(cv_image,(a[0],a[1]), 60, (0,0,255), -1) #pixelpair a
        cv2.circle(cv_image,(b[0],b[1]), 60, (0,0,255), -1) #pixelpair b
        cv2.circle(cv_image,(c[0],c[1]), 60, (0,0,255), -1) #pixelpair c
        cv2.circle(cv_image,(d[0],d[1]), 60, (0,0,255), -1) #pixelpair d
        cv2.circle(cv_image,(e[0],e[1]), 60, (0,0,255), -1) #pixelpair e
        basename = "log_image"
        suffix = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
        filename = "_".join([basename, suffix])
        filename = filename + '.png'
        path = os.getcwd() + '/' + filename
        cv2.imwrite(filename, cv_image)
        retrieve_depth = True   #Let the depth image callback know we have pixels to calculate on
        
        print('callback done, calculating depth')
        
        #time.sleep(1)

def depth_callback(data):
    global right_pos, retrieve_depth, xc_pix, yc_pix, depth_done, X, Y, Z 
    
    pub_coord = rospy.Publisher('coord', Float32MultiArray, queue_size=1)

    if retrieve_depth == True:
        xci = data.width / 2
        yci = data.height / 2
        #f = 500 # focal length in pixels HD720 resolution ZED2
        f = 700 # focal length ZED1
        b = 0.12 # baseline
        Pxi = int(xc_pix)
        Pyi = int(yc_pix)
      
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='32FC1')

        #pixelpair a
        coo_az = cv_image[a[1], a[0]]*1000*(yci-a[1])/f
        coo_ay = cv_image[a[1], a[0]]*1000*(xci-a[0])/f
        coo_ax = np.sqrt((cv_image[a[1], a[0]]*1000)**2 - coo_ay**2 - coo_az**2) 

        #pixelpair b
        coo_bz = cv_image[b[1], b[0]]*1000*(yci-b[1])/f
        coo_by = cv_image[b[1], b[0]]*1000*(xci-b[0])/f
        coo_bx = np.sqrt((cv_image[b[1], b[0]]*1000)**2 - coo_by**2 - coo_bz**2) 

        #pixelpair c
        coo_cz = cv_image[c[1], c[0]]*1000*(yci-c[1])/f
        coo_cy = cv_image[c[1], c[0]]*1000*(xci-c[0])/f
        coo_cx = np.sqrt((cv_image[c[1], c[0]]*1000)**2 - coo_cy**2 - coo_cz**2) 

        #pixelpair d
        coo_dz = cv_image[d[1], d[0]]*1000*(yci-d[1])/f
        coo_dy = cv_image[d[1], d[0]]*1000*(xci-d[0])/f
        coo_dx = np.sqrt((cv_image[d[1], d[0]]*1000)**2 - coo_dy**2 - coo_dz**2) 

        #pixelpair e
        coo_ez = cv_image[e[1], e[0]]*1000*(yci-e[1])/f
        coo_ey = cv_image[e[1], e[0]]*1000*(xci-e[0])/f
        coo_ex = np.sqrt((cv_image[e[1], e[0]]*1000)**2 - coo_ey**2 - coo_ez**2) 
  
        retrieve_depth = False
        
        arr = [coo_ax, coo_ay, coo_az, coo_bx, coo_by, coo_bz, coo_cx, coo_cy, coo_cz, coo_dx, coo_dy, coo_dz, coo_ex, coo_ey, coo_ez]
        coord_fma = Float32MultiArray(data=arr)
        pub_coord.publish(coord_fma)
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


def transform(current, camera):
    theta = np.arctan(float(current[1])/float(current[0]))
    
    Z = current[2] - camera[0] + 200 #200 mm margin from camera to target on ground
    X = current[0] + camera[1]*np.cos(theta) + camera[2]*np.sin(theta)
    Y = current[1] + camera[1]*np.sin(theta) - camera[2]*np.cos(theta)
    
    return X, Y, Z

def coord_callback(data):
    global Xa, Ya, Za
    for i in [0, 3, 6, 9, 12]:
        print('------------Pixel pair: ', i, '------------')
        print('Camera coordinates: ', data[i], data[i+1], data[i+2])
        print('Global coordinates: ', transform([Xa, Ya, Za], [data[i], data[i+1], data[i+2]]))

def main():
    rospy.init_node('master_node', anonymous=False)
    #rate = rospy.Rate(0.015)
    rate = rospy.Rate(0.1)
    rospy.wait_for_service('go_to_target')
    rospy.wait_for_service('get_pos')

    directory = '/home/jetson/run_inf_jet'
    os.chdir(directory)

    rospy.Subscriber('/zed/zed_node/left/image_rect_color',Image,callback)
    rospy.Subscriber("arm_BP", Bool, callback_bool)
    rospy.Subscriber('/zed/zed_node/depth/depth_registered',Image, depth_callback)
    rospy.Subscriber('coord', Float32MultiArray, coord_callback)

    goto = rospy.ServiceProxy('go_to_target',GoToTarget)

    pub_BP = rospy.Publisher('arm_BP', Bool, queue_size=1)



    A = [2200, 0, 1100]
    global Xa, Ya, Za
    while not rospy.is_shutdown():
        print('-----------Starting test---------------')
        rospy.wait_for_service('go_to_target')
        response = goto(A[0], A[1], A[2])
        Xa = response.x_current
        Ya = response.y_current
        Za = response.z_current
        print('Response: ', response.x_current, response.y_current, response.z_current)
        pub_BP.publish(True)

        
        print('code done, wait for rate')
        rate.sleep()





    rospy.spin()

if __name__ == '__main__':
    main()
