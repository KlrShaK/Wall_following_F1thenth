#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input


angle_range = 360		# sensor angle range of the lidar
car_length = 1.5	# projection distance we project car forward. 
vel = 1.5 		# used for pid_vel (not much use).
error = 0.0
dist_from_wall = 0.8

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
	theta = 55
	left_dist = float(getRange(data, 270))
	a = getRange(data,(90 + theta)) # Ray at degree theta from right_dist
	right_dist = getRange(data,90)	# Ray perpendicular to right side of car
	theta_r = math.radians(theta)
	# dist_from_wall = (left_dist + right_dist)/2 # keep in middle

	# Calculating the deviation of steering(alpha)
	alpha = math.atan( (a * math.cos(theta_r) - right_dist)/ a * math.sin(theta_r) )
	AB = right_dist * math.cos(alpha) # Present Position
	CD = AB + car_length * math.sin(alpha) # projection in Future Position

	error = dist_from_wall - CD # total error
	# print('error: ', error) #Testing

	# Sending PID error to Control
	msg = pid_input()
	msg.pid_error = error
	msg.pid_vel = vel 
	pub.publish(msg)
	

if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("scan",LaserScan,callback)
	rospy.spin()
