#! /usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from pal_robotiq_gripper_wrapper import gripper_grasp

if __name__ == '__main__':
    rospy.init_node('gripper_grasping')
    gg = gripper_grasp.GripperGrasp()
    rospy.spin()
