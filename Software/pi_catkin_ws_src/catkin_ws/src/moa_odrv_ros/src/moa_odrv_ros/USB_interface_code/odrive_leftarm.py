#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import rospy
import std_msgs.msg
from std_msgs.msg import Float64, Int32, Float32MultiArray, Float64MultiArray
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
import std_srvs.srv

import diagnostic_updater, diagnostic_msgs.msg

import odrive
from odrive.enums import (
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_IDLE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
)
from odrive.utils import *
import math
import traceback


def check_and_print_err(odr, axis_index, odrive_name, axis_name):
    try:
        if axis_index == 0:
            axis = odr.axis0
        else:
            axis = odr.axis1
    except:
        rospy.logerr(type(odr))
        quit()

    if axis.error != 0:
        rospy.logerr(
            "Failed calibration with "+str(odrive_name)+":"+str(axis_name)+" error " +str(axis.error)+", motor error "+str(axis.motor.error)
        )
        dump_errors(odr)
        quit()


class OdrivePositionControl:
    def __init__(self):
        self.lArmMsg = Float64MultiArray()
        self.lArmMsg.data = 5 * [0]
        # self.rArmMsg = Float64MultiArray()
        # self.rArmMsg.data = 5 * [0]
        print("finding an odrive...")
        # self.R1 = odrive.find_any(serial_number="20643681424D")
        # print("found R1")
        # self.R2 = odrive.find_any(serial_number="2055367B424D")
        # print("found R2")
        # self.R3 = odrive.find_any(serial_number="20603681424D")
        # print("found R3")
        self.L1 = odrive.find_any(serial_number="20583681424D")
        print("found L1")
        self.L2 = odrive.find_any(serial_number="2061389C304E")
        print("found L2")
        # self.L3 = odrive.find_any(serial_number="2051367C424D")
        # print("found L3")
        self.stop()
        self.clear_error()


        print("subscribed")
        print("starting calibration...")
        self.L1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.L1.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.L2.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.L2.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        rospy.sleep(0.5)
        # print(self.R1)
        # print(self.R2)
        # print(self.R3)
        # print(self.R1.axis0.current_state != AXIS_STATE_IDLE)
        # while (self.R1.axis0.current_state != AXIS_STATE_IDLE) or (self.R1.axis1.current_state != AXIS_STATE_IDLE) or (self.R2.axis0.current_state != AXIS_STATE_IDLE) or (self.R2.axis1.current_state != AXIS_STATE_IDLE) or (self.R3.axis0.current_state != AXIS_STATE_IDLE):
        rate = rospy.Rate(10)
        while (
            (self.L1.axis0.current_state != AXIS_STATE_IDLE)
            or (self.L1.axis1.current_state != AXIS_STATE_IDLE)
            or (self.L2.axis0.current_state != AXIS_STATE_IDLE)
            or (self.L2.axis1.current_state != AXIS_STATE_IDLE)
        ):

            check_and_print_err(self.L1, 0, "L1", "axis0")
            check_and_print_err(self.L1, 1, "L1", "axis1")
            check_and_print_err(self.L2, 0, "L2", "axis0")
            check_and_print_err(self.L2, 1, "L2", "axis1")
            rate.sleep()

        self.clear_error()
        self.engage()
        print("Odrives Initilization Complete.")

        rospy.Subscriber(
            "/leftarm/position_controller/command",
            Float64MultiArray,
            self.leftarmcallback,
        )
    def leftarmcallback(self, lArmMsg):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", lArmMsg.data)
        try:
            self.L1.axis0.controller.input_pos = lArmMsg.data[0]
            # rospy.sleep(0.1)
            self.L1.axis1.controller.input_pos = lArmMsg.data[1]
            rospy.sleep(0.1)
            self.L2.axis0.controller.input_pos = lArmMsg.data[2]
            # rospy.sleep(0.1)
            self.L2.axis1.controller.input_pos = lArmMsg.data[3]
            rospy.sleep(0.1)
            # self.L3.axis0.controller.input_pos = lArmMsg.data[4]
            # rospy.sleep(0.1)
        except:
            rospy.loginfo(50 * "#" + " L1 " + 50 * "#")
            dump_errors(self.L1, True)
            rospy.loginfo(50 * "#" + " L2 " + 50 * "#")
            dump_errors(self.L2, True)
    def checkerr(self):
        if self.L1.axis0.error != 0:
            print(
                "Failed calibration with axis0 error 0x%x, motor error 0x%x"
                % (self.L1.axis0.error, self.L1.axis0.motor.error)
            )
            self.clear_error()
            return False
        elif self.L1.axis1.error != 0:
            print(
                "Failed calibration with axis1 error 0x%x, motor error 0x%x"
                % (self.L1.axis1.error, self.L1.axis1.motor.error)
            )
            self.clear_error()
            return False
        elif self.L2.axis1.error != 0:
            print(
                "Failed calibration with axis1 error 0x%x, motor error 0x%x"
                % (self.L2.axis0.error, self.L2.axis0.motor.error)
            )
            self.clear_error()
            return False
        elif self.L2.axis1.error != 0:
            print(
                "Failed calibration with axis1 error 0x%x, motor error 0x%x"
                % (self.L2.axis1.error, self.L2.axis1.motor.error)
            )
            self.clear_error()
            return False
        else:
            return True

    def engage(self):
        self.L1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.L1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.L2.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.L2.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def stop(self):
        self.L1.axis0.requested_state = AXIS_STATE_IDLE
        self.L1.axis1.requested_state = AXIS_STATE_IDLE
        self.L2.axis0.requested_state = AXIS_STATE_IDLE
        self.L2.axis1.requested_state = AXIS_STATE_IDLE

    def clear_error(self):
        self.L1.clear_errors()
        self.L2.clear_errors()

class ROSLogger(object):
    """Imitate a standard Python logger, but pass the messages to rospy logging."""

    def debug(self, msg):
        rospy.logdebug(msg)  #  print(msg) #

    def info(self, msg):
        rospy.loginfo(msg)  #  print(msg) #

    def warn(self, msg):
        rospy.logwarn(msg)  #  print(msg) #

    def error(self, msg):
        rospy.logerr(msg)  #  print(msg) #

    def critical(self, msg):
        rospy.logfatal(msg)  #  print(msg) #


def get_param(name, default):
    val = rospy.get_param(name, default)
    rospy.loginfo("  %s: %s", name, str(val))
    return val

    # Or to change a value, just assign to the property
    # R1_odrv.axis0.controller.input_pos = 1
    # print("Position setpoint is " + str(R1_odrv.axis0.controller.pos_setpoint))

    # And this is how function calls are done:
    # for i in [1,2,3,4]:
    # print('voltage on GPIO{} is {} Volt'.format(i, R1_odrv.get_adc_voltage(i)))


def main():
    rospy.init_node("left_arm_odrive_position_control")
    opc = OdrivePositionControl()
    rospy.spin()


def lim_range(x, out_min, out_max):
    if x > out_max:
        return out_max
    elif x < out_min:
        return out_min
    else:
        return x


def map_values(x, in_min, in_max, out_min, out_max):
    if x < 0:
        in_max = 0
        out_max = 0
    else:
        in_min = 0
        out_min = 0
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
