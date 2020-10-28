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



right_pos = False

## Data structure for Image:
# type(data) = class sensor_msgs.msg._Image.Image
# data.header gives the header
# data.height 720 (uint32)
# data.width 1280 (uint32)
# data.encoding bgra8
# data.is_bigendian 0 (False)
# data.step 5120 (uint32)
# data.data: len=3 686 400, type='bytes'

class Server:
    def __init__(self):
        self.imagemsg = None
        self.pointcloudmsg = None
        self.armposmsg = None

    def image_callback(self, msg):
        self.imagemsg = msg
        #rospy.loginfo('image_to_coord -> image callback heard encoding: %s ', msg.encoding)
        self.compute()

    def pointcloud_callback(self, msg):
        self.pointcloudmsg = msg
        rospy.loginfo('image_to_coord -> pointcloud callback heard is_dense: %s ', msg.header)
        self.compute()

    def armposmsg_callback(self, msg):
        self.armposemsg = msg
        #rospy.loginfo('image_to_coord -> arm_BP callback heard:  %s ', msg.data)
        self.compute()
        

    def compute(self):
        if self.imagemsg is not None and self.pointcloudmsg is not None and self.armposmsg is not None:
            pass


def main():
    
    rospy.init_node('image_to_coord', anonymous=False)

    server = Server()

    rospy.Subscriber('/zed2/zed_node/left/image_rect_color',Image, server.image_callback)
    rospy.Subscriber("arm_BP", Bool, server.armposmsg_callback)
    rospy.Subscriber('/zed2/zed_node/point_cloud/cloud_registered', PointCloud2, server.pointcloudmsg)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    main()
