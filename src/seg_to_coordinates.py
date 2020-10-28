#!/usr/bin/env python


## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import struct
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image

def get_pixel_pair(seg_data):
    #print("doing some nasty stuff to the picture data") 
    rospy.loginfo('seg_to_coordinates -> get pixel pair is working on data')
    pixels = [300, 500]
    
    ## TODO add the actual function for getting pixel pairs out of segmented area
   
    return pixels

def get_3d_coord(pixels):
    #print('getting coordinates in 3D')
    rospy.loginfo('seg_to_coordinates -> get_3d_coord is working on pixels: %s', pixels)
    return [1, 2, 3]



def callback(data):
    
    rospy.loginfo('seg_to_coordinates -> segmented_area callback heard: %s ', data.data)
    # http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
    #print(data.data)
    #data_unpacked = struct.unpack('<I', data.data[0])
    #data_unpacked = int.from_bytes(data.data, byteorder='little')
    #print(data_unpacked)

    
    coord_pub = rospy.Publisher('coord_3D', Float32MultiArray, queue_size=1)
    seg_data = [4, 6, 8]
    pixs = get_pixel_pair(seg_data)
    coord3D = get_3d_coord(pixs)
    coord_fma = Float32MultiArray(data=coord3D)
    
    coord_pub.publish(coord_fma)

def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('seg_to_coordinates', anonymous=False)

    #rospy.Subscriber('coord_3D', Float32MultiArray, callback_point)
    #rospy.Subscriber('/zed2/zed_node/point_cloud/cloud_registered', PointCloud2, callback_point)
    rospy.Subscriber('segmented_area', Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
