#!/usr/bin/env python

import rclpy.cre
import sys
# import actionlib
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.duration import Duration

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

closed  = [0.0]

if __name__ == "__main__":

  rclpy.init(args=sys.argv)
  node = rclpy.create_node('home_gripper')
  suffix = node.declare_parameter("~suffix", Parameter.Type.STRING, None)
  if suffix == None:
      node.get_logger("No suffix found in param: ~suffix")
      exit(1)
  joint_names = ["gripper" + suffix + "_finger_joint"]
  node.get_logger("Waiting for gripper" + suffix + "_controller...")
  client = ActionClient(node, FollowJointTrajectoryAction, "gripper" + suffix + "_controller/follow_joint_trajectory")
  client.wait_for_server()
  node.get_logger("...connected.")
  
  # TODO: use wait_for_message (not available for humble) 
  # rclpy.wait_for_message(JointState, "joint_states")

  trajectory = JointTrajectory()
  trajectory.joint_names = joint_names
  trajectory.points.append(JointTrajectoryPoint())
  trajectory.points[0].positions = closed
  trajectory.points[0].velocities = [0.0 for i in joint_names]
  trajectory.points[0].accelerations = [0.0 for i in joint_names]
  trajectory.points[0].time_from_start = rclpy.Duration(2.0)

  node.get_logger("Opening gripper...")
  goal = FollowJointTrajectoryGoal()
  goal.trajectory = trajectory
  goal.goal_time_tolerance = rclpy.Duration(0.0)

  client.send_goal(goal)
  client.wait_for_result(rclpy.Duration(3.0))
  node.get_logger("Gripper opened.")
