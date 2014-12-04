#!/usr/bin/env python

import sys

import roslib;
import rospy
from geometry_msgs.msg import Twist
from ros_arduino_msgs.msg import *

lastError = 0
valordeseado = 3500
# KP = 13
# KD = 1
KO = 190.0
KP = 8
KD = 10

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist)

def main():
	rospy.init_node('arduino_linefollower')
	rospy.loginfo("linefollower started")

	rospy.Subscriber("/arduino/sensor/line_follower", Analog, lfCallback)

	while not rospy.is_shutdown():
		pass

def lfCallback(req):
	global lastError

	posicion = req.value
	print posicion
	twist = Twist()
	error = posicion - valordeseado

	error /= 100

	#twist.linear.x = 0.1*(1/error)
	v_ang = (KP*error + KD*(error - lastError))/KO
	twist.linear.x = 0.085
	lastError = error
	twist.angular.z = v_ang
	rospy.loginfo(v_ang)
	cmd_vel_pub.publish(twist)



main()
