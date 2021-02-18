#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input


angle_range = 360		# sensor angle range of the lidar
car_length = 1.5	# projection distance we project car forward. 
vel = 1.0 		# used for pid_vel (not much use).
error = 0.0

pub = rospy.Publisher('error', pid_input, queue_size=10)

def getRange(data,theta):

    angle_range = data.angle_max - data.angle_min
    angle_increment  = data.angle_increment
    scan_range = []
    rad2deg_factor = 57.296
    angle_range *= rad2deg_factor
    angle_increment *= rad2deg_factor

    for element in data.ranges:
        if math.isnan(element) or math.isinf(element):
            element = 100
        scan_range.append(element)

    index = round(theta / angle_increment)

    return scan_range[index]

def callback(data):

    dist_in_front = getRange(data, 180)
    theta = 30
    left_dist = getRange(data, 270)
    right_dist = getRange(data,90)	# Ray perpendicular to right side of car
    a_right = getRange(data,(90 + theta)) # Ray at degree theta from right_dist
    a_left = getRange(data,(270 - theta))
    theta = math.radians(theta)

    # Calculating the deviation of steering(alpha) from right
    alpha_r = math.atan( (a_right * math.cos(theta) - right_dist)/ a_right * math.sin(theta) )
    curr_pos_r = right_dist * math.cos(alpha_r) # Present Position
    fut_pos_r = curr_pos_r + car_length * math.sin(alpha_r) # projection in Future Position

    # Calculating the deviation of steering(alpha) from left
    alpha_l = math.atan( (a_left * math.cos(theta) - left_dist)/ a_left * math.sin(theta) )
    curr_pos_l = left_dist * math.cos(alpha_l) # Present Position
    fut_pos_l = curr_pos_l + car_length * math.sin(alpha_l) # projection in Future Position

    error = - (fut_pos_r - fut_pos_l) # total error
    # print('error: ', error) #Testing

    # Sending PID error to Control
    msg = pid_input()
    msg.pid_error = error
    msg.pid_vel = dist_in_front # pid_vel used for distance in front
    pub.publish(msg)


if __name__ == '__main__':
    print("Laser node started")
    rospy.init_node('dist_finder',anonymous = True)
    rospy.Subscriber("scan",LaserScan,callback)
    rospy.spin()
