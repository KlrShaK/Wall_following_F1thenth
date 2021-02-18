#!/usr/bin/env python3

import rospy
import math
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDriveStamped

kp = 0.27# 0.27 for porto
kd = 0.0125#0.001 #0.09
ki = 0 #0.5
kp_vel = 0.9 #9 is using keep in middle
kd_vel = 0.0001
ki_error = 0
prev_error = 0.0 
vel_input = 2.5	# base velocity
angle = 0.0	# initial steering angle

pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=5)


def control(data):
	global prev_error
	global vel_input
	global kp
	global kp_vel
	global kd
	global kd_vel
	global ki
	global angle

	dist_in_front = data.pid_vel
	front_obs = 1
	e = data.pid_error
	# Calculating deviation for lateral deviation from path
	kp_error = kp * e
	kd_error = kd * (e - prev_error)

	# Calculating error for velocity
	kp_vel_er = kp_vel * e
	kd_vel_er = kd * (e - prev_error)
	# ki_error = ki_error + (ki * e)

	vel_error = kp_vel_er + kd_vel_er
	pid_error = kp_error + kd_error


	min_angle=-20
	max_angle= 20

	# Heigher error results in lower velocity
	# while lower error results in heigher velocity
	if dist_in_front <= 5:
		front_obs = 3#abs(- ( (math.log10(dist_in_front/5)/math.log10(2)) +1 ))
	velo = vel_input + 1/ ( (abs(vel_error)*front_obs ))
	print("velo :", velo)
	#corrected steering angle
	angle = pid_error

	#print("raw velo:", velo) # Testing

	#Speed limit
	if velo > 50 :
		velo = 50

	# Filtering steering angle for Out-of-Range values
	if angle < min_angle:
		angle = min_angle
	elif angle > max_angle:
		angle = max_angle

	# print("filtered angle :" , angle) # Testing


	# Sending Drive information to Car
	msg = AckermannDriveStamped()
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = ''
	msg.drive.speed = velo
	msg.drive.steering_angle = angle
	pub.publish(msg)

if __name__ == '__main__':
	print("Starting control...")
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
