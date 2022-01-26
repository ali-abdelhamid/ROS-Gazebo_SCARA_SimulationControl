#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import time
from gazebo_msgs.srv import *

def j1Controller():
	kp = 5.0
	kd = 8.0
	rate = 0.01
	j1R = float(sys.argv[1]) # target j1 position
	
	rospy.wait_for_service('gazebo/get_joint_properties')
	print("Looking for service...")
	get_joint_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties)
	print("Service Found")
	j1M = float(get_joint_properties("joint1").position[0]) # measured j1 position
	j1Mi = j1M # initial measured j1 position
	print("initial j1 position: ",j1Mi)
	
	i= 0
	error_i = 0
	error  = j1R - j1M
	while (error != 0):
		error = j1R - j1M
		errorDot = (error - error_i)/rate
		u = kd*errorDot + kp*error
		print("joint effort: ", u)
		rospy.wait_for_service('gazebo/apply_joint_effort')
		apply_joint_effort = rospy.ServiceProxy('gazebo/apply_joint_effort', ApplyJointEffort,persistent=False)
		apply_joint_effort(joint_name="joint1",effort=u,duration=rospy.Time(rate))
		j1Mi = j1M
		error_i = error
		rospy.wait_for_service('gazebo/get_joint_properties')
		get_joint_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties,persistent=False)
		j1M = float(get_joint_properties("joint1").position[0])  # measured j1 position
		print("j1M: ", j1M)
		
		f = open("jointPositions.txt","a")
		f.write(str(j1M))
		f.write(" ")
		g = open("time.txt","a")
		g.write(str(i*rate))
		g.write(" ")
		
		i += 1
		time.sleep(rate)
	return 0
	
def zController():
	kp = 5.0
	kd = 8.0
	rate = 1
	j3R = float(sys.argv[1]) # target j3 position
	
	rospy.wait_for_service('gazebo/get_joint_properties')
	print("Looking for service...")
	get_joint_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties)
	print("Service Found")
	j3M = float(get_joint_properties("joint3").position[0]) # measured j3 position
	j3Mi = j3M # initial measured j3 position
	print("initial j3 position: ",j3Mi)
	
	i= 0
	error_i = 0
	error  = j3R - j3M
	while (error != 0):
		error = j3R - j3M
		print('Error: ', error)
		errorDot = (error - error_i)/rate
		u = kd*errorDot + kp*error
		print("joint effort: ", u)
		rospy.wait_for_service('gazebo/apply_joint_effort')
		apply_joint_effort = rospy.ServiceProxy('gazebo/apply_joint_effort', ApplyJointEffort,persistent=False)
		apply_joint_effort(joint_name="joint3",effort=u,duration=rospy.Time(rate))
		j3Mi = j3M
		error_i = error
		rospy.wait_for_service('gazebo/get_joint_properties')
		get_joint_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties,persistent=False)
		j3M = float(get_joint_properties("joint3").position[0])  # measured j3 position
		print("j3M: ", j3M)
		
		f = open("jointPositions.txt","a")
		f.write(str(j3M))
		f.write(" ")
		g = open("time.txt","a")
		g.write(str(i*rate))
		g.write(" ")
		
		i += 1
		time.sleep(rate)
	return 0

if __name__ == "__main__":
	print('Test Test')
	zController()
	print("Test Complete")
