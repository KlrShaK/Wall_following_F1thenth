#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    print(data.ranges[270])

rospy.init_node("scan_test", anonymous=False)
sub = rospy.Subscriber("/scan", LaserScan, callback)
rospy.spin()
