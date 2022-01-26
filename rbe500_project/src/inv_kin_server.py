#!/usr/bin/env python

from __future__ import print_function

from rbe500_project.srv import invKin,invKinResponse
import rospy
import math

def handle_invKin(req):
    print("Your end effector is at (x,y,z): ",(req.x, req.y, req.z))
    x = req.x
    y = req.y
    z = req.z
    
    c = (pow(x,2) + pow(y,2) - 1 - 1)/2
    s = pow(1-pow(c,2),0.5)
    
    j1 = math.atan2(x,y) - math.atan2(1+c,s)
    j2 = math.atan2(c,pow(1-pow(c,2),0.5))
    j3 = 1.0 - z
    
    return invKinResponse(j1,j2,j3)

def inv_kin_server():
    rospy.init_node('inv_kin_server')
    s = rospy.Service('inv_kin_server', invKin, handle_invKin)
    print("Ready to calculate inverse kinematics.")
    rospy.spin()

if __name__ == "__main__":
    inv_kin_server()

