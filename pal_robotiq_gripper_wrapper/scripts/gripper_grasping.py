#!/usr/bin/env python

"""
@author: Sai Kishor Kothakota

Grasp controller to close with a determined error on position only
so to skip overheating.

"""

import rclpy
import time
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String, UInt8, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import JointState
from rcl_interfaces import ParameterType
from rcl_interfaces.msg import SetParametersResult

from rclpy.parameter import Parameter


class GripperGrasp(Node):
    def parameters_callback(self, params):
        for param in params:
            if param.name == "timeout":
                self.timeout = param.value
            elif param.name == "closing_time":
                self.closing_time = param.value
            elif param.name == "rate":
                self.rate = param.value
            elif param.name == "pressure":
                self.pressure_configuration = param.value
        return SetParametersResult(successful=True)
    
    def __init__(self):
        super().__init__('gripper_grasping')
        self.get_logger().info("Initializing Gripper Grasper...")

        # Get the params from param server
        self.last_state = None
        self.on_optimal_close = False
        self.on_optimal_open = False
        self.is_grasped_msg = Bool()

        # Check if simulation is running
        self.use_sim_time = self.declare_parameter(
            "/use_sim_time", Parameter.Type.BOOL, False).value

        self.controller_name = self.declare_parameter(
            "~controller_name", Parameter.Type.STRING, None).value

        if not self.controller_name:
            self.get_logger().error("No controller name found in param: ~controller_name")
            self.destroy_node()
        self.real_joint_names = self.declare_parameter(
            "~real_joint_names", Parameter.Type.STRING, None).value
        if not self.real_joint_names:
            self.get_logger().error("No real joint names given in param: ~real_joint_names")
            self.destroy_node()
        self.close_configuration = self.declare_parameter(
            "~close_configuration", Parameter.Type.DOUBLE_ARRAY, None).value
        if not self.close_configuration:
            self.get_logger().error(
                "No close configuration values given in param: ~close_configuration")
            self.destroy_node()
        if len(self.close_configuration) != len(self.real_joint_names):
            self.get_logger().error("The close configuration values defined in param: ~close_configuration"
                                    ", should be of same length as that of joints in param: ~real_joint_names")
            self.destroy_node()

        
        # self.timeout = self.declare_parameter("timeout", 2.0,
        #                                       "timeout fort to use dynamic parameters in my ROS2 code. I defined a callback function, if a parameter is changed with following line: the closing action", )
        #DYNAMIC!!!
        self.timeout = self.declare_parameter("timeout", Parameter.Type.DOUBLE, 2.0).value


        self.closing_time = self.declare_parameter("closing_time", Parameter.Type.DOUBLE, 0.2).value
        self.rate = self.declare_parameter("rate",Parameter.Type.INTEGER, 25).value
        self.pressure_configuration = self.declare_parameter("pressure",Parameter.Type.DOUBLE, 0.08).value
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("Initialized dynamic reconfigure on: " + str(rclpy.get_name()))

        # Set opening time of gripper
        self.opening_time = 0.02

        # Subscriber to the gripper state
        self.state_sub = self.create_subscription(JointTrajectoryControllerState, 
                                                  self.controller_name + 
                                                  '_controller/state', '/' + self.state_cb, 1)
        self.get_logger().info("Subscribed to topic: " + str(self.state_sub.resolved_name))

        # Publisher on the gripper command topic
        self.cmd_pub = self.create_publisher(JointTrajectory, '/' + self.controller_name 
                                             + '_controller/command', queue_size=1)
        self.get_logger().info("Publishing to topic: " + str(self.cmd_pub.resolved_name))

        # Grasping service to offer
        self.grasp_srv = self.create_service(Empty, '/' + self.controller_name + 
                                             '_controller/grasp', self.grasp_cb)
        self.get_logger().info("Offering grasp service on: " +
                      str(self.grasp_srv.resolved_name))
        self.get_logger().info("Done initializing Gripper Grasp Service!")

        # Release service to offer
        self.release_srv = self.create_service(Empty, '/' + self.controller_name +
                                               '_controller/release', self.release_cb)
        self.get_logger().info("Offering release service on: " +
                      str(self.release_srv.resolved_name))
        self.get_logger().info("Done initializing Gripper Release Service!")

        # Publish a human readable status of the gripper (Only available in non simulation)
        if not self.use_sim_time:
            self.gripper_motor_name = self.declare_parameter(
                "~gripper_motor_name", None).value
            self.sub_gs = self.create_subscription(UInt8, 
                                                   "{}/gripper_status".format(self.gripper_motor_name),
                                                    self.grip_status_cb, 1)
            self.pub_gth = self.create_publisher(String, 
                                                 "{}/gripper_status_human".format(self.gripper_motor_name), 1)

        self.pub_js = self.create_publisher(Bool, "{}/is_grasped".format(self.controller_name),
                                            1)


    def ddr_cb(self, config, level):
        self.get_logger().info(
            "timeout" + str(config['timeout']) + "closing_time" + str(config['closing_time']))
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

        # If in simulation
        if self.use_sim_time:
            self.last_state = data
            if self.on_optimal_close is True:
                self.is_grasped_msg.data = True
                if self.last_state.error.positions[0] <= 0.01:
                    self.is_grasped_msg.data = False
                    self.on_optimal_close = False
            else:
                self.is_grasped_msg.data = False
            self.pub_js.publish(self.is_grasped_msg)
        # if real robot
        else:
            self.last_state = data

    def check_is_grasped(self):
        # If in simulation
        if self.use_sim_time:
            if self.last_state.error.positions[0] >= 0.03:
                time.sleep(1)
            if self.last_state.error.positions[0] >= 0.03:
                return True
            return False
        # if real robot
        else:
            return self.is_grasped_msg.data

    def grasp_cb(self, req):
        self.get_logger().debug("Received grasp request!")
        # From wherever we are close gripper
        self.on_optimal_open = False
        # Keep closing until the error of the state reaches
        # max_position_error on any of the gripper joints
        # or we reach timeout

        initial_time = rclpy.Time.now()
        r = rclpy.Rate(self.rate)
        closing_amount = self.close_configuration
        # Initial command, wait for it to do something
        self.get_logger().info("Closing: " + str(closing_amount))
        self.send_joint_trajectory(
            closing_amount, self.closing_time, self.on_optimal_close)
        while not rclpy.is_shutdown() and \
            (rclpy.Time.now() - initial_time) < rclpy.Duration(self.timeout) and \
                not self.on_optimal_close and self.last_state:
            if self.check_is_grasped():
                closing_amount = [
                    self.last_state.actual.positions[0]+self.pressure_configuration]
                self.on_optimal_close = True
                self.get_logger().info("Closing: " + str(closing_amount))
                self.send_joint_trajectory(
                    closing_amount, self.closing_time, self.on_optimal_close)
            time.sleep(self.rate)

        return EmptyResponse()

    def release_cb(self, req):
        self.get_logger().debug("Received release request!")
        # From wherever we are opening gripper
        self.on_optimal_close = False

        open_amount = [0.0] * len(self.real_joint_names)
        # Initial command, wait for it to do something
        if self.on_optimal_open is False:
            self.get_logger().info("Opening: " + str(open_amount))
            self.send_joint_trajectory(
                open_amount, self.opening_time, self.on_optimal_open)
            self.on_optimal_close = False
            self.on_optimal_open = True
            time.sleep(self.opening_time)

        return EmptyResponse()

    def send_joint_trajectory(self, joint_positions, execution_time, goal_position_reached):
        jt = JointTrajectory()
        jt.joint_names = self.real_joint_names
        jt.header.stamp = rclpy.Time.now()
        p = JointTrajectoryPoint()
        p.positions = joint_positions

        # on_optimal_position can be either for on_optimal_close or on_optimal_open
        if goal_position_reached == True:
            # Duration after grasping
            p.time_from_start = rospy.Duration(0.2)
        else:
            # Duration of the grasping
            p.time_from_start = rospy.Duration(execution_time)
        jt.points.append(p)

        self.cmd_pub.publish(jt)

    def grip_status_cb(self, data):
        # publish data to topic translated to human understanding
        bin_number = bin(data.data)[2:].zfill(8)  # data range: 0 -> 255
        gOBJ = hex(int(bin_number[:2], 2))
        gSTA = hex(int(bin_number[2:4], 2))
        gGTO = hex(int(bin_number[4], 2))
        gACT = hex(int(bin_number[7], 2))
        if(str(gOBJ) == "0x1" or str(gOBJ) == "0x2"):
            self.is_grasped_msg.data = True
        else:
            if(str(gOBJ) == "0x0" and self.on_optimal_close is True):
                self.is_grasped_msg.data = True
            else:
                self.is_grasped_msg.data = False
                self.on_optimal_close = False
        self.pub_js.publish(self.is_grasped_msg)

        self.pub_gth.publish("Gripper status: " +
                             self.hex_to_human(gOBJ, gSTA, gGTO, gACT))

    def hex_to_human(self, gOBJ, gSTA, gGTO, gACT):
        gOBJ_dict = {"0x0": "Fingers are in motion towards requested position. No object detected",
                     "0x1": "Fingers have stopped due to a contact while opening before requested position. Object detected opening",
                     "0x2": "Fingers have stopped due to a contact while closing before requested position. Object detected closing",
                     "0x3": "Fingers are at requested position. No object detected or object has been loss / dropped"}

        gSTA_dict = {"0x0": "Gripper is in reset ( or automatic release ) state. See Fault Status if gripper is activated",
                     "0x1": "Activation in progress",
                     "0x2": "Not used",
                     "0x3": "Activation is completed"}

        gGTO_dict = {"0x0": "Stopped (or performing activation / automatic release)",
                     "0x1": "Go to Position Request"}

        gACT_dict = {"0x0": "Gripper reset",
                     "0x1": "Gripper activation"}
        try:
            res = gACT_dict[gACT] + " " + gGTO_dict[gGTO] + \
                " " + gSTA_dict[gSTA] + " " + gOBJ_dict[gOBJ]
        except Exception:
            self.get_logger().info(
                "Not able to decode hex codes in gOBJ, gSTA, gGTO, gACT")
            self.get_logger().info("gOBJ hex: " + str(gOBJ))
            self.get_logger().info("gSTA hex: " + str(gSTA))
            self.get_logger().info("gGTO hex: " + str(gGTO))
            self.get_logger().info("gACT hex: " + str(gACT))
            res = None
        return res


if __name__ == '__main__':

    rclpy.init(args=sys.argv)
    gg = GripperGrasp()
    rclpy.spin(gg)
    # rclpy.shutdown()
