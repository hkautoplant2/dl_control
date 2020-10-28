#!/usr/bin/env python


## Subscribes to ZED and retrieves the IMU data

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

def callback(data):
    #print('Starting hearing this -----------------------------')
    rospy.loginfo(rospy.get_caller_id() + 'IMU_subscriber heard orientation: \r\n %s', data.orientation)
    

def main():
	# Main node, this is a listener/subscriber node
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('imu_subscriber', anonymous=True)

    rospy.Subscriber('/zed2/zed_node/imu/data', Imu, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
