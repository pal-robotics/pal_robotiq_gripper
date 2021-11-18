#!/usr/bin/env python

"""
@author: Sai Kishor Kothakota

Grasp controller to close with a determined error on position only
so to skip overheating.

"""

import rospy
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String, UInt8
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import JointState
from pal_robotiq_gripper_wrapper_msgs.msg import GripperStatus
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
                  not on_optimal_close and self.last_state:
            for index in range(len(self.last_state.error.positions)):
                if self.last_state.error.positions[index] > self.max_position_error:
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


class GripperGraspStatus(object):
    def __init__(self):
        rospy.loginfo("Initializing Gripper Grasper Status ...")
        # Publish a human readable status of the gripper
        self.gripper_motor_name = rospy.get_param("~gripper_motor_name", None)
        self.sub_gs = rospy.Subscriber("{}/gripper_status".format(self.gripper_motor_name),
                         UInt8, self.grip_status_cb, queue_size=1)
        self.pub_gth = rospy.Publisher("{}/gripper_status_human".format(self.gripper_motor_name), String, queue_size=1)
        # Publish gripper state (position and current) and translates position to real distance between fingers (robotiq gripper 85)
        self.sub_js = rospy.Subscriber("joint_states", JointState, self.joint_state_cb, queue_size=1)
        self.ee = rospy.get_param("~model")
        self.gripper_joint_name = rospy.get_param("~gripper_joint_name")
        self.pub_js = rospy.Publisher("{}/state".format(self.gripper_joint_name.replace('_finger', '')), GripperStatus, queue_size=1)

    def grip_status_cb(self, data):
        # publish data to topic translated to human understanding
        bin_number = bin(data.data)[2:].zfill(8)  # data range: 0 -> 255
        gOBJ = hex(int(bin_number[:2],2))
        gSTA = hex(int(bin_number[2:4],2))
        gGTO = hex(int(bin_number[4],2))
        gACT = hex(int(bin_number[7],2))

        rospy.loginfo("Gripper status: " + self.hex_to_human(gOBJ, gSTA, gGTO, gACT))
        self.pub_gth.publish("Gripper status: " + self.hex_to_human(gOBJ, gSTA, gGTO, gACT))

    def hex_to_human(self, gOBJ, gSTA, gGTO, gACT):
        gOBJ_dict = {"0x0": "Fingers are in motion towards requested position. No object detected",
                     "0x1": "Fingers have stopped due to a contact while opening before requested position. Object detected opening",
                     "0x2": "Fingers have stopped due to a contact while closing before requested position. Object detected closing",
                     "0x3": "Fingers are at requested position. No object detected or object has been loss / dropped"}

        gSTA_dict = {"0x0": "Gripper is in reset ( or automatic release ) state. See Fault Status if gripper is activated",
                     "0x1": "Activation in progress", "0x2": "Not used", "0x3": "Activation is completed"}

        gGTO_dict = {"0x0": "Stopped (or performing activation / automatic release)",
                     "0x1": "Go to Position Request"}

        gACT_dict = {"0x0": "Gripper reset",
                     "0x1": "Gripper activation"}
        try:
            res = gACT_dict[gACT] + " " + gGTO_dict[gGTO] + " " + gSTA_dict[gSTA] + " " + gOBJ_dict[gOBJ]
        except Exception:
            rospy.logerr("Not able to decode hex codes in gOBJ, gSTA, gGTO, gACT")
            rospy.logerr("gOBJ hex: "+str(gOBJ))
            rospy.logerr("gSTA hex: "+str(gSTA))
            rospy.logerr("gGTO hex: "+str(gGTO))
            rospy.logerr("gACT hex: "+str(gACT))
            res = None
        return res

    def joint_state_cb(self, data):
        gfj_index = data.name.index(self.gripper_joint_name)
        gripper_status_msg = GripperStatus()
        gripper_status_msg.header = data.header
        gripper_status_msg.name = data.name[gfj_index]
        gripper_status_msg.position = data.position[gfj_index]
        gripper_status_msg.fingers_distance = self.gripper_pos_to_dist(data.position[0])
        gripper_status_msg.effort = data.effort[gfj_index]
        self.pub_js.publish(gripper_status_msg)

    def gripper_pos_to_dist(self, pos):
        
        if self.ee == "robotiq-2f-85":
            # Empiric formula https://docs.google.com/spreadsheets/d/1UbA8CLmDVlxi3S_ETz7UTv8AqJj86mhsNu1IXPgQgMk/edit#gid=0
            res = (-113*pos + 87)/100 # cm to m
        else:
            res = pos
        return res
    


if __name__ == '__main__':
    rospy.init_node('gripper_grasping')
    gg = GripperGraspService()
    gg_stat = GripperGraspStatus()
    rospy.spin()
