#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import time
import math
from multiprocessing import Process
import sys
from gazebo_msgs.srv import *

j1R = float(sys.argv[1]) # target joint 1 position, radians
j2R = float(sys.argv[2]) # target joint 2 position, radians
j3R = float(sys.argv[3]) # target joint 3 position, radians

""" uncomment this section to use cartesian inputs instead of joint inputs
a1 = 2 # link 1 length
a2 = 2 # link 2 length

x = float(sys.argv[1]) # target x position
y = float(sys.argv[2]) # target y position
z = float(sys.argv[3]) # target z position

c = (pow(x,2) + pow(y,2) - pow(a1,2) - pow(a2,2))/(2*a1*a2) # ignore; abstraction used in inverse kinematics
s = pow(1-pow(c,2),0.5) # ignore; abstraction used in inverse kinematics
    
j1R = math.atan2(y,x) - math.atan2(a1 + a2*2,a2*s) # joint 1 reference position
j2R = math.atan2(c,pow(1-pow(c,2),0.5)) # joint 2 reference position
j3R = 2.0 - z # joint 3 reference position
"""

def j1Controller():
	kp = 5.0
	kd = 8.0
	rate = 0.01
	
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
	while (1):
		error = j1R - j1M
		errorDot = (error - error_i)/rate
		u = kd*errorDot + kp*error
		rospy.wait_for_service('gazebo/apply_joint_effort')
		apply_joint_effort = rospy.ServiceProxy('gazebo/apply_joint_effort', ApplyJointEffort,persistent=False)
		apply_joint_effort(joint_name="joint1",effort=u,duration=rospy.Time(rate))
		j1Mi = j1M
		error_i = error
		rospy.wait_for_service('gazebo/get_joint_properties')
		get_joint_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties,persistent=False)
		j1M = float(get_joint_properties("joint1").position[0])  # measured j1 position
		rospy.wait_for_service('gazebo/get_model_state')
		get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState,persistent=False)
		print('~~~~~~~~~')
		print('adjusting joint 1 to ', j1R)
		print('current joint value is ', j1M)
		# print('cartesian position')
		# print(get_model_state(model_name='my_robot',relative_entity_name='my_robot::link4').pose.position)
			
		f = open("joint1Positions.txt","a")
		f.write(str(j1M))
		f.write(" ")
		g = open("time1.txt","a")
		g.write(str(i*rate))
		g.write(" ")
		
		i += 1
		time.sleep(rate)
	return 0
	
def j2Controller():
	kp = 5.0
	kd = 8.0
	rate = 0.01

	rospy.wait_for_service('gazebo/get_joint_properties')
	print("Looking for service...")
	get_joint_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties)
	print("Service Found")
	j2M = float(get_joint_properties("joint2").position[0]) # measured j2 position
	j2Mi = j2M # initial measured j2 position
	print("initial j2 position: ",j2Mi)

	i= 0
	error_i = 0
	error  = j2R - j2M
	while (1):
		error = j2R - j2M
		errorDot = (error - error_i)/rate
		u = kd*errorDot + kp*error
		rospy.wait_for_service('gazebo/apply_joint_effort')
		apply_joint_effort = rospy.ServiceProxy('gazebo/apply_joint_effort', ApplyJointEffort,persistent=False)
		apply_joint_effort(joint_name="joint2",effort=u,duration=rospy.Time(rate))
		j2Mi = j2M
		error_i = error
		rospy.wait_for_service('gazebo/get_joint_properties')
		get_joint_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties,persistent=False)
		j2M = float(get_joint_properties("joint2").position[0])  # measured j2 position
		rospy.wait_for_service('gazebo/get_model_state')
		get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState,persistent=False)
		print('~~~~~~~~~')
		print('adjusting joint 2 to ', j2R)
		print('current joint value is ', j2M)
		# print('cartesian position')
		# print(get_model_state(model_name='my_robot',relative_entity_name='my_robot::link4').pose.position)
		
		f = open("joint2Positions.txt","a")
		f.write(str(j2M))
		f.write(" ")
		g = open("time2.txt","a")
		g.write(str(i*rate))
		g.write(" ")
		
		i += 1
		time.sleep(rate)
	return 0

	
def zController():
	kp = 5.0
	kd = 8.0
	rate = 0.01

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
	while (1):
		error = j3R - j3M
		errorDot = (error - error_i)/rate
		u = kd*errorDot + kp*error
		rospy.wait_for_service('gazebo/apply_joint_effort')
		apply_joint_effort = rospy.ServiceProxy('gazebo/apply_joint_effort', ApplyJointEffort,persistent=False)
		apply_joint_effort(joint_name="joint3",effort=u,duration=rospy.Time(rate))
		j3Mi = j3M
		error_i = error
		rospy.wait_for_service('gazebo/get_joint_properties')
		get_joint_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties,persistent=False)
		j3M = float(get_joint_properties("joint3").position[0])  # measured j3 position
		rospy.wait_for_service('gazebo/get_model_state')
		get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState,persistent=False)
		print('~~~~~~~~~')
		print('adjusting joint 3 to ', j3R)
		print('current joint value is ', j3M)
		# print('cartesian position')
		# print(get_model_state(model_name='my_robot',relative_entity_name='my_robot::link4').pose.position)
		
		f = open("joint3Positions.txt","a")
		f.write(str(j3M))
		f.write(" ")
		g = open("time3.txt","a")
		g.write(str(i*rate))
		g.write(" ")
		
		i += 1
		time.sleep(rate)
	return 0

if __name__ == "__main__":
	print('Beep Boop')
	print('Adjusting joint 1')
	p1 = Process(target = j1Controller)
	p1.start()
	print('Adjusting joint 2')
	p2 = Process(target = j2Controller)
	p2.start()
	print('Adjusting joint 3')
	p3 = Process(target = zController)
	p3.start()
