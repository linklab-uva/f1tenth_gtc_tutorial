#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from simulator.msg import PIDInput
import pdb

ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 1.0 #0.9 # meters
DESIRED_DISTANCE_LEFT = 0.8 # 0.55
VELOCITY = 1.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

pub = rospy.Publisher('error', PIDInput, queue_size=10)

def getRange(data, angle):
	# data: single message from topic /scan
	# angle: between -45 to 225 degrees, where 0 degrees is directly to the right
	# Outputs length in meters to object with angle in lidar scan field of view
	if angle > 179.9:
		angle = 179.9
	index = len(data.ranges) * (angle + 45) / ANGLE_RANGE
	dist = data.ranges[int(index)]
	if math.isinf(dist):
		return 10.0
	if math.isnan(dist):
		return 4.0
	return data.ranges[int(index)]

def followRight(data, desired_trajectory):
	# data: single message from topic /scan
	# desired_trajetory: desired distance to the right wall [meters]
	global alpha

	a = getRange(data,60)
	b = getRange(data,0)
	swing = math.radians(60)
	alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	print "a","b", a, b
	print "Alpha right",math.degrees(alpha)
	curr_dist = b*math.cos(alpha)

	future_dist = curr_dist + CAR_LENGTH * math.sin(alpha)
	print "Right : ",future_dist
	error = desired_trajectory - future_dist

	print "Current Distance Right: ", curr_dist
	return error, curr_dist

def followLeft(data, desired_trajectory):
	# data: single message from topic /scan
	# desired_trajectory: desired distance to the left wall [meters]
	global alpha

	a = getRange(data,120)
	b = getRange(data,179.9)
	swing = math.radians(60)
	print "a","b", a, b
	alpha = -math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	print "Alpha left",math.degrees(alpha)
	curr_dist = b*math.cos(alpha)

	future_dist = curr_dist - CAR_LENGTH * math.sin(alpha)
	print "Left : ",future_dist
	error = future_dist - desired_trajectory

	print "Current Distance Left: ", curr_dist
	return error, curr_dist

def followCenter(data):
	# data: single message from topic /scan
	global alpha

	a = getRange(data,120)
	b = getRange(data,179.9)
	swing = math.radians(60)
	print "center distances: ", a, b
	alpha = -math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	print "Alpha left",math.degrees(alpha)
	curr_dist1 = b*math.cos(alpha)
	future_dist1 = curr_dist1-CAR_LENGTH*math.sin(alpha)

	a = getRange(data,60)
	b = getRange(data,0)
	swing = math.radians(60)
	alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	print "Alpha right",math.degrees(alpha)
	curr_dist2 = b*math.cos(alpha)
	future_dist2 = curr_dist2+CAR_LENGTH*math.sin(alpha)

	print "dist 1 : ",future_dist1
	print "dist 2 : ",future_dist2

	error = future_dist1 - future_dist2
	print "Error : ",error
	return error, curr_dist2 - curr_dist1

def callback(data):
	global error
	global alpha
	global final_direction
	global prev_direction

	print " "

	# Does a left wall follow
	error_left, curr_dist_left = followLeft(data, DESIRED_DISTANCE_LEFT)
	error = error_left

	# Does a right wall follow
#	error_right, curr_dist_right = followRight(data, DESIRED_DISTANCE_RIGHT)
#	error = error_right

	# This is code bock for center wall follow
#	error_center, curr_dist_center = followCenter(data)
#	error = error_center

	msg = PIDInput()
	msg.pid_error = error
	msg.pid_vel = VELOCITY
	pub.publish(msg)


if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("scan",LaserScan,callback)
	rospy.spin()
