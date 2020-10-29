#!/usr/bin/env python


## Description:
# This node get_image subscribes to the topics arm_BP and one of the image topics from ZED. When the arm_BP callback signals True, the image callback goes into the function "do_something".

import roslib
roslib.load_manifest('dl_control')
import rospy
import cv2
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

    def image_callback(self, msg):
        #rospy.loginfo('image_to_coord -> image callback heard encoding: %s ', msg.encoding)
        self.imagemsg = msg
        
        self.compute()

    def pointcloud_callback(self, msg):
        #rospy.loginfo('image_to_coord -> pointcloud callback heard is_dense: %s  ', msg.is_dense)
        self.pointcloudmsg = msg

        self.compute()

    def armposmsg_callback(self, msg):
        rospy.loginfo('image_to_coord -> arm_BP callback heard:  %s ', msg.data)
        self.armposmsg = msg
        
        self.compute()
        

    def compute(self):
        if not (self.imagemsg == None and self.pointcloudmsg == None and self.armposmsg == None):
            if self.armposmsg.data == True:
                point_cloud = gen_points(self.pointcloudmsg)
               
                print point_cloud[600:605, 300:305]


def main():
    
    rospy.init_node('image_to_coord', anonymous=False)

    server = Server()

    rospy.Subscriber('/zed2/zed_node/left/image_rect_color',Image, server.image_callback)
    rospy.Subscriber('/arm_BP', Bool, server.armposmsg_callback)
    rospy.Subscriber('/zed2/zed_node/point_cloud/cloud_registered', PointCloud2, server.pointcloud_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    main()
