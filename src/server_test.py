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

def paint(pic):
  
# Polygon corner points coordinates 
    pts = np.array([[25, 70], [25, 160],  
                [110, 200], [200, 160],  
                [200, 70], [110, 20]], 
               np.int32) 
  
    pts = pts.reshape((-1, 1, 2)) 
  
    isClosed = True

    color = (255, 0, 0) 

    thickness = 2

    image = cv2.polylines(pic, [pts],  
                      isClosed, color, thickness) 
    return image

def callback(data):
    rospy.loginfo('--test -> pointcloud callback heard is_dense: %s ', data.header)
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    painted = paint(cv_image)
    cv2.imshow("Image window", painted)
    cv2.waitKey(3)


def main():
    
    rospy.init_node('image_to_coord', anonymous=False)

    rospy.Subscriber('/zed2/zed_node/point_cloud/cloud_registered', PointCloud2, server.pointcloudmsg)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    main()
