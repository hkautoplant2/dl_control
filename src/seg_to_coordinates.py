#!/usr/bin/env python


## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import struct
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

def get_pixel_pair(seg_data):

    #rospy.loginfo('seg_to_coordinates -> get pixel pair received data: %s ', seg_data)
    pixels = [160, 440]
    
    ## TODO add the actual function for getting pixel pairs out of segmented area
   
    return pixels

def get_3d_coord(pixels):
    #print('getting coordinates in 3D')
    #rospy.loginfo('seg_to_coordinates -> get_3d_coord is working on pixels: %s', pixels)
    return [1, 2, 3]



def callback(data):
    
    rospy.loginfo('seg_to_coordinates -> segmented_area callback heard: %s ', data.data)
    
    coord_pub = rospy.Publisher('coord_3D', Float32MultiArray, queue_size=1)
    vis_pub = rospy.Publisher('visual_done', Bool, queue_size=1)
    #seg_data = [4, 6, 8]
    pixs = get_pixel_pair(data.data)
    coord3D = get_3d_coord(pixs)
    coord_fma = Float32MultiArray(data=coord3D)
    coord_pub.publish(coord_fma)
    vis_pub.publish(True)

def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('seg_to_coordinates', anonymous=False)

    rospy.Subscriber('dnn_output', Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
