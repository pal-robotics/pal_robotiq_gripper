#!/usr/bin/env python

"""
@author: Sai Kishor Kothakota

Grasp controller to close with a determined error on position only
so to skip overheating.

"""
import rospy
from std_msgs.msg import String, UInt8MultiArray
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Bool, Float64
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


class GripperGrasp(object):
    def __init__(self):
        # Init Gripper grasp node
        rospy.loginfo("Initializing Gripper Grasper...")


        self.on_optimal_close = False
        self.is_grasped_msg = Bool()

        # Get the params from param server
        self.controller_name = rospy.get_param("~controller_name", None)
        if not self.controller_name:
            rospy.logerr("No controller name found in param: ~controller_name")
            exit(1)

    

        # Publish a human readable status of the vacuum gripper
        self.gripper_motor_name = rospy.get_param("~gripper_motor_name", None)
        self.sub_gs = rospy.Subscriber("{}/gripper_status".format(self.gripper_motor_name),
                         UInt8MultiArray, self.grip_status_cb, queue_size=1)
        self.pub_gth = rospy.Publisher("{}/gripper_status_human".format(self.gripper_motor_name), String, queue_size=1)

        # Publish a boolean to know if an object is grasped or not
        self.pub_js = rospy.Publisher("{}/is_grasped".format(self.controller_name), Bool , queue_size=1)

        # This node Dynamic params
        self.ddr = DDynamicReconfigure(
            self.controller_name + "_grasp_service")
        self.rate = self.ddr.add_variable("rate",
                                  "Rate Hz at which the node closing will do stuff",
                                  25, 10, 50)
        self.pressure_configuration = self.ddr.add_variable("pressure",
                                          "Requested vacuum/pressure",
                                          1.0, 0.1, 1.0)
        self.ddr.start(self.ddr_cb)
        # Publisher on the gripper command topic
        self.cmd_pub = rospy.Publisher('/' + self.controller_name + '_controller/command',
                                       Float64,
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

        # Release service to offer
        self.release_srv = rospy.Service('/' + self.controller_name + '_controller/release',
                                       Empty,
                                       self.release_cb)
        rospy.loginfo("Offering release service on: " +
                      str(self.release_srv.resolved_name))
        rospy.loginfo("Done initializing Gripper Release Service!")


    def ddr_cb(self, config, level):
        self.pressure_configuration = config['pressure']
        self.rate = config['rate']
        return config

    def grasp_cb(self, req):
        rospy.logdebug("Received grasp request!")
        # Desactivate the vacuum gripper
        self.on_optimal_close = False
        self.send_command(0.0)
        initial_time = rospy.Time.now()
        r = rospy.Rate(self.rate)
        pressure_amount = self.pressure_configuration
        # Activate the vacuum gripper
        self.send_command(pressure_amount)
        while not rospy.is_shutdown() and (rospy.Time.now() - initial_time) < rospy.Duration(3.0):
            if self.is_grasped_msg.data is True:
                self.on_optimal_close = True
            r.sleep()
        # Desactivate the vacuum gripper if nothing is grasped after Timeout
        if self.on_optimal_close == False:
            self.send_command(0.0)
        return EmptyResponse()
    
    def release_cb(self, req):
        rospy.logdebug("Received release request!")
        # Desactivate the vacuum gripper
        self.send_command(0.0)
        return EmptyResponse()


    def send_command(self, pressure_amount):
        rospy.loginfo("Requested vacuum/pressure: " + str(pressure_amount))
        vacuum_msg = Float64()
        vacuum_msg.data = pressure_amount
        self.cmd_pub.publish(vacuum_msg)


    def grip_status_cb(self, data):
        # publish data to topic translated to human understanding
        if isinstance(data.data[0], bytes):
            bin_number = bin(bytearray(data.data)[0])[2:].zfill(8)  # data range: 0 -> 255
        else:
            bin_number = bin(data.data[0])[2:].zfill(8) # data range: 0 -> 255
        gOBJ = hex(int(bin_number[:2],2))
        gSTA = hex(int(bin_number[2:4],2))
        gGTO = hex(int(bin_number[4],2))
        gACT = hex(int(bin_number[7],2))
        if(str(gOBJ) == "0x1" or str(gOBJ) == "0x2"):
            self.is_grasped_msg.data = True
        else:
            if self.on_optimal_close is True:
                self.on_optimal_close = False
                self.send_command(0.0) #Release after the object is droped or released
            self.is_grasped_msg.data = False
        self.pub_js.publish(self.is_grasped_msg)

        self.pub_gth.publish("Gripper status: " + self.hex_to_human(gOBJ, gSTA, gGTO, gACT))



    def hex_to_human(self, gOBJ, gSTA, gGTO, gACT):
        gOBJ_dict = {"0x0": "Unknown object detection. Regulating towards requested vacuum/pressure",
                     "0x1": "Object detected. Minimum vacuum value reached",
                     "0x2": "Object detected. Maximum vacuum value reached",
                     "0x3": "No object detected. Object loss, dropped or gripping timeout reached"}

        gSTA_dict = {"0x0": "Gripper is not activated",
                     "0x3": "Gripper is operational"}

        gGTO_dict = {"0x0": "Stop the vacuum generator; valves are in position to hold the workpiece",
                     "0x1": "Follow the requested vacuum parameters in real time"}

        gACT_dict = {"0x0": "Clear Gripper fault status",
                     "0x1": "Gripper is operational"}
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
