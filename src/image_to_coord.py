#!/usr/bin/env python


## Description:


import roslib
roslib.load_manifest('dl_control')
import rospy
import cv2
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import time
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import os
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
import struct
from sensor_msgs import point_cloud2



right_pos = False

# Test reshape for our data
    #testlist = [[1, 2, 3], [2, 3, 4], [3, 4, 5], [1, 1, 1], [2,2,2], [3,3,3]]
    #testarr = np.array(testlist).reshape((2, 3, 3))
    #print('testlist: ', testlist)
    #print('arr: ', testarr)
   # print(testarr[0, 1])

## Data structure for Image:
# type(data) = class sensor_msgs.msg._Image.Image
# data.header gives the header
# data.height 720 (uint32)
# data.width 1280 (uint32)
# number of pixels in one image = 921 600
# data.encoding bgra8
# data.is_bigendian 0 (False)
# data.step 5120 (uint32)
# data.data: len=3 686 400, type='bytes'

def gen_points(msg):
    assert isinstance(msg, PointCloud2)
    cloud_points = list(point_cloud2.read_points(msg, skip_nans=False, field_names = ("x", "y", "z")))
    cloud_points = np.array(cloud_points).reshape((msg.height, msg.width, 3))

    return cloud_points


class Server:
    def __init__(self):
        self.imagemsg = None
        self.pointcloudmsg = None
        self.armposmsg = None
        self.pixelpair = None
        self.segmentations = None
        self.coord_3D = None

    def image_callback(self, msg):
        #rospy.loginfo('image_to_coord -> image callback heard encoding: %s ', msg.encoding)
        self.imagemsg = msg
        ## TODO Add dnn which outputs the segmentation
        self.segmentations = self.dnn()
        #self.compute()

    def pointcloud_callback(self, msg):
        #rospy.loginfo('image_to_coord -> pointcloud callback heard is_dense: %s  ', msg.is_dense)
        self.pointcloudmsg = msg

        #self.compute()

    def armposmsg_callback(self, msg):
        #rospy.loginfo('image_to_coord -> arm_BP callback heard:  %s ', msg.data)
        self.armposmsg = msg
        
        #self.compute()
        

    def compute(self):
        if not (self.imagemsg == None and self.pointcloudmsg == None and self.armposmsg == None):
            if self.armposmsg.data == True:
                point_cloud = gen_points(self.pointcloudmsg)     
                pixel_matrix = point_cloud[x, y]
                #print pixel_matrix
                ## TODO 

    def dnn(self):
        #print('DNN throughput')
        return [1, 2, 3]
 

    def depth_callback(self, msg):
        #rospy.loginfo('image_to_coord -> pointcloud callback heard is_dense: %s  ', msg.data)
        cx = msg.width / 2
        cy = msg.height / 2
        f = 500 # focal length in pixels HD720 resolution
        b = 0.12 # baseline
        x = cx-200 
        y = cy -200
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        if math.isnan(cv_image[y, x]) == False and math.isinf(cv_image[y, x]) == False:
            Zp = cv_image[y, x]
            Yp = (y - cy) * Zp / f 
            Xp = (x - cx) * Zp / f
            print('point', Xp, Yp, Zp)
            self.coord_3D = (Xp, Yp, Zp)
         
        


def main():
    
    rospy.init_node('image_to_coord', anonymous=False)

    server = Server()

    rospy.Subscriber('/zed2/zed_node/left/image_rect_color',Image, server.image_callback)
    rospy.Subscriber('/arm_BP', Bool, server.armposmsg_callback)
    rospy.Subscriber('/zed2/zed_node/point_cloud/cloud_registered', PointCloud2, server.pointcloud_callback)

    rospy.Subscriber('/zed2/zed_node/depth/depth_registered',Image, server.depth_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    main()
