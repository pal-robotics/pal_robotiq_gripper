#!/usr/bin/env python

"""
@author: Sai Kishor Kothakota

Grasp controller to close with a determined error on position only
so to skip overheating.

"""

import rospy
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty, EmptyResponse
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


class GripperGraspService(object):
    def __init__(self):
        rospy.loginfo("Initializing Gripper Grasper...")

        # Get the params from param server
        self.last_state = None
        self.controller_name = rospy.get_param("~controller_name", None)
        if not self.controller_name:
            rospy.logerr("No controller name found in param: ~controller_name")
            exit(1)
        self.real_joint_names = rospy.get_param("~real_joint_names", None)
        if not self.real_joint_names:
            rospy.logerr(
                "No real joint names given in param: ~real_joint_names")
            exit(1)
        self.close_configuration = rospy.get_param(
            "~close_configuration", None)
        if not self.close_configuration:
            rospy.logerr(
                "No close configuration values given in param: ~close_configuration")
            exit(1)
        if len(self.close_configuration) != len(self.real_joint_names):
            rospy.logerr(
                "The close configuration values defined in param: ~close_configuration"
                ", should be of same length as that of joints in param: ~real_joint_names")
            exit(1)

        # This node Dynamic params
        self.ddr = DDynamicReconfigure(
            self.controller_name + "_grasp_service")
        self.max_position_error = self.ddr.add_variable("max_position_error",
                                                        "Max absolute value of controller "
                                                        "state of any joint to stop closing",
                                                        0.002, 0.00001, 0.045)
        self.timeout = self.ddr.add_variable("timeout",
                                             "timeout for the closing action",
                                             5.0, 0.0, 30.0)
        self.closing_time = self.ddr.add_variable("closing_time",
                                                  "Time for the closing goal",
                                                  2.0, 0.01, 30.0)
        self.rate = self.ddr.add_variable("rate",
                                          "Rate Hz at which the node closing will do stuff",
                                          5, 1, 50)

        self.ddr.start(self.ddr_cb)
        rospy.loginfo("Initialized dynamic reconfigure on: " +
                      str(rospy.get_name()))

        # Subscriber to the gripper state
        self.state_sub = rospy.Subscriber('/' + self.controller_name + '_controller/state',
                                          JointTrajectoryControllerState,
                                          self.state_cb,
                                          queue_size=1)
        rospy.loginfo("Subscribed to topic: " +
                      str(self.state_sub.resolved_name))

        # Publisher on the gripper command topic
        self.cmd_pub = rospy.Publisher('/' + self.controller_name + '_controller/command',
                                       JointTrajectory,
                                       queue_size=1)
        rospy.loginfo("Publishing to topic: " +
                      str(self.cmd_pub.resolved_name))

        # Grasping service to offer
        self.grasp_srv = rospy.Service('/' + self.controller_name + '_controller/grasp',
                                       Empty,
                                       self.grasp_cb)
        rospy.loginfo("Offering grasp service on: " +
                      str(self.grasp_srv.resolved_name))
        rospy.loginfo("Done initializing Gripper Grasp Service!")

    def ddr_cb(self, config, level):
        self.max_position_error = config['max_position_error']
        self.timeout = config['timeout']
        self.closing_time = config['closing_time']
        self.rate = config['rate']
        return config

    def state_cb(self, data):
        self.last_state = data

    def grasp_cb(self, req):
        rospy.logdebug("Received grasp request!")
        # From wherever we are close gripper

        # Keep closing until the error of the state reaches
        # max_position_error on any of the gripper joints
        # or we reach timeout
        initial_time = rospy.Time.now()
        closing_amount = self.close_configuration
        # Initial command, wait for it to do something
        self.send_close(closing_amount)
        rospy.sleep(self.closing_time)
        r = rospy.Rate(self.rate)
        on_optimal_close = False
        while not rospy.is_shutdown() and \
                  (rospy.Time.now() - initial_time) < rospy.Duration(self.timeout) and \
                  not on_optimal_close:
            for index in range(len(self.last_state.error.positions)):
                if -self.last_state.error.positions[index] > self.max_position_error:
                    rospy.logdebug("Over error joint {}...".format(index))
                    closing_amount = self.get_optimal_close()
                    on_optimal_close = True
            self.send_close(closing_amount)
            r.sleep()

        return EmptyResponse()

    def get_optimal_close(self):
        optimal_state = []
        for pos in self.last_state.actual.positions:
            optimal_state.append(pos - self.max_position_error)
        return optimal_state

    def send_close(self, closing_amount):
        rospy.loginfo("Closing: " + str(closing_amount))
        jt = JointTrajectory()
        jt.joint_names = self.real_joint_names
        p = JointTrajectoryPoint()
        p.positions = closing_amount
        p.time_from_start = rospy.Duration(self.closing_time)
        jt.points.append(p)

        self.cmd_pub.publish(jt)


if __name__ == '__main__':
    rospy.init_node('gripper_grasping')
    gg = GripperGraspService()
    rospy.spin()
