#!/usr/bin/env python

"""
@author: Sai Kishor Kothakota

Grasp controller to close with a determined error on position only
so to skip overheating.

"""
import rospy
from pal_robotiq_gripper_wrapper import gripper_grasp

if __name__ == '__main__':
    rospy.init_node('gripper_grasping')
    gg = gripper_grasp.GripperGraspSim()
    rospy.spin()
