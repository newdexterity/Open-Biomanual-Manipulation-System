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
        self.rArmMsg = Float64MultiArray()
        self.rArmMsg.data = 5 * [0]
        print("finding an odrive...")
        self.R3 = odrive.find_any(serial_number="20603681424D")
        print("found R3")
        self.L3 = odrive.find_any(serial_number="2051367C424D")
        print("found L3")
        self.stop()
        self.clear_error()

        print("subscribed")
        print("starting calibration...")
        self.R3.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.L3.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        rospy.sleep(0.5)
        # print(self.R1)
        # print(self.R2)
        # print(self.R3)
        # print(self.R1.axis0.current_state != AXIS_STATE_IDLE)
        # while (self.R1.axis0.current_state != AXIS_STATE_IDLE) or (self.R1.axis1.current_state != AXIS_STATE_IDLE) or (self.R2.axis0.current_state != AXIS_STATE_IDLE) or (self.R2.axis1.current_state != AXIS_STATE_IDLE) or (self.R3.axis0.current_state != AXIS_STATE_IDLE):
        rate = rospy.Rate(10)
        while (
            (self.R3.axis0.current_state != AXIS_STATE_IDLE)
            or (self.L3.axis0.current_state != AXIS_STATE_IDLE)
        ):
            check_and_print_err(self.R3, 0, "R3", "axis0")
            check_and_print_err(self.L3, 0, "L3", "axis0")
            rate.sleep()

        self.clear_error()
        self.engage()
        print("Odrives Initilization Complete.")
        # rospy.sleep(10)

        rospy.Subscriber(
            "/leftarm/position_controller/command",
            Float64MultiArray,
            self.leftarmcallback,
        )
        rospy.Subscriber(
            "/rightarm/position_controller/command",
            Float64MultiArray,
            self.rightarmcallback,
        )

    def leftarmcallback(self, lArmMsg):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", lArmMsg.data)
        try:
            self.L3.axis0.controller.input_pos = lArmMsg.data[4]
            rospy.sleep(0.1)
        except:
            rospy.loginfo(50 * "#" + " L3 " + 50 * "#")
            dump_errors(self.L3, True)

    def rightarmcallback(self, rArmMsg):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", rArmMsg.data)
        try:
            self.R3.axis0.controller.input_pos = rArmMsg.data[4]
            rospy.loginfo(50 * "#" + " R3 " + 50 * "#")
            dump_errors(self.R3, True)
        except:
            rospy.loginfo(50 * "#" + " R3 " + 50 * "#")
            dump_errors(self.R3, True)
        # self.checkerr()

    def checkerr(self):
        if self.R3.axis0.error != 0:
            print(
                "Failed calibration with axis1 error 0x%x, motor error 0x%x"
                % (self.R3.axis0.error, self.R3.axis0.motor.error)
            )
            self.clear_error()
            return False
        elif self.L3.axis0.error != 0:
            print(
                "Failed calibration with axis1 error 0x%x, motor error 0x%x"
                % (self.L3.axis0.error, self.L3.axis0.motor.error)
            )
            self.clear_error()
            return False
        else:
            return True

    def engage(self):
        if not self.R1:
            print("Not connected.")
            return False
        self.R3.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.L3.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def stop(self):
        self.R3.axis0.requested_state = AXIS_STATE_IDLE
        self.L3.axis0.requested_state = AXIS_STATE_IDLE

    def clear_error(self):
        self.R3.clear_errors()
        self.L3.clear_errors()


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
    rospy.init_node("odrive_position_control")
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
