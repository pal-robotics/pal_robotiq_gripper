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
from std_msgs.msg import Bool
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


class GripperGrasp(object):
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

        self.timeout = self.ddr.add_variable("timeout",
                                             "timeout for the closing action",
                                             2.0, 2.0, 5.0)
        self.closing_time = self.ddr.add_variable("closing_time",
                                                  "Time for the closing goal",
                                                  0.2, 0.01, 5.0)
        self.rate = self.ddr.add_variable("rate",
                                          "Rate Hz at which the node closing will do stuff",
                                          25, 10, 50)

        self.pressure_configuration = self.ddr.add_variable("pressure",
                                          "Aditional distance to apply more or less pressure",
                                          0.08, 0.05, 0.2)

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
        self.on_optimal_close = False

        # Init Grasp Status
        # TODO: Enable current functionality if tiago dual and both ee are robotiq-2f
        # Publish a human readable status of the gripper
        self.gripper_motor_name = rospy.get_param("~gripper_motor_name", None)
        self.sub_gs = rospy.Subscriber("{}/gripper_status".format(self.gripper_motor_name),
                         UInt8, self.grip_status_cb, queue_size=1)
        self.pub_gth = rospy.Publisher("{}/gripper_status_human".format(self.gripper_motor_name), String, queue_size=1)
        # Publish a boolean to know if an object is grasped or not
        self.pub_js = rospy.Publisher("{}/is_grasped".format(self.gripper_motor_name), Bool , queue_size=1)
        rospy.loginfo(rospy.get_param("pal_robot_info/type"))
        self.tiago_type = "tiago"
        self.robotiq_side = ""
        if rospy.get_param("pal_robot_info/type") == "tiago_dual":
            self.tiago_type = "tiago_dual"
            for side in ("right","left"):
                if "robotiq-2f" in rospy.get_param("pal_robot_info/end_effector_"+side):
                    self.ee = rospy.get_param("pal_robot_info/end_effector_"+side)
                    self.robotiq_side = "_"+side
        else:
            self.ee = rospy.get_param("pal_robot_info/end_effector")

        self.is_grasped_msg = Bool()

    def ddr_cb(self, config, level):
        rospy.loginfo("timeout" + str(config['timeout']) + "closing_time" + str(config['closing_time']))
        if config['timeout'] - 2 <= config['closing_time']:
            self.timeout = config['closing_time'] + 2
            config['timeout'] = config['closing_time'] + 2
        else:
            self.timeout = config['timeout']
        self.closing_time = config['closing_time']
        self.pressure_configuration = config['pressure']
        self.rate = config['rate']
        return config

    def state_cb(self, data):
        self.last_state = data

    def grasp_cb(self, req):
        rospy.logdebug("Received grasp request!")
        # From wherever we are close gripper
        self.on_optimal_close = False
        # Keep closing until the error of the state reaches
        # max_position_error on any of the gripper joints
        # or we reach timeout
        initial_time = rospy.Time.now()
        r = rospy.Rate(self.rate)
        closing_amount = self.close_configuration
        # Initial command, wait for it to do something
        self.send_close(closing_amount)
        while not rospy.is_shutdown() and \
                  (rospy.Time.now() - initial_time) < rospy.Duration(self.timeout) and \
                  not self.on_optimal_close and self.last_state:
            if self.is_grasped_msg.data is True:
                closing_amount = [self.last_state.actual.positions[0]+self.pressure_configuration]
                self.on_optimal_close = True
                self.send_close(closing_amount)
            r.sleep()

        return EmptyResponse()

    def send_close(self, closing_amount):
        rospy.loginfo("Closing: " + str(closing_amount))
        jt = JointTrajectory()
        jt.joint_names = self.real_joint_names
        jt.header.stamp = rospy.Time.now()
        p = JointTrajectoryPoint()
        p.positions = closing_amount
        if self.on_optimal_close == True:
            # Duration after grasping
            p.time_from_start = rospy.Duration(0.2)
        else:
            # Duration of the grasping
            p.time_from_start = rospy.Duration(self.closing_time)
        jt.points.append(p)

        self.cmd_pub.publish(jt)
        

    def grip_status_cb(self, data):
        # publish data to topic translated to human understanding
        bin_number = bin(data.data)[2:].zfill(8)  # data range: 0 -> 255
        gOBJ = hex(int(bin_number[:2],2))
        gSTA = hex(int(bin_number[2:4],2))
        gGTO = hex(int(bin_number[4],2))
        gACT = hex(int(bin_number[7],2))
        if(str(gOBJ) == "0x1" or str(gOBJ) == "0x2"):
            self.is_grasped_msg.data = True
        else:
            if(str(gOBJ) == "0x0" and self.on_optimal_close is True):
                self.is_grasped_msg.data = True
            else:
                self.is_grasped_msg.data = False
                self.on_optimal_close = False
        self.pub_js.publish(self.is_grasped_msg)

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
            rospy.logerr("gOBJ hex: " + str(gOBJ))
            rospy.logerr("gSTA hex: " + str(gSTA))
            rospy.logerr("gGTO hex: " + str(gGTO))
            rospy.logerr("gACT hex: " + str(gACT))
            res = None
        return res


if __name__ == '__main__':
    rospy.init_node('gripper_grasping')
    gg = GripperGrasp()
    rospy.spin()
