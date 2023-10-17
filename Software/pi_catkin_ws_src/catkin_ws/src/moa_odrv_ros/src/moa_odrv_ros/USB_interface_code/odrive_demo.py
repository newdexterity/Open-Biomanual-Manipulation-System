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
from odrive.enums import *
from odrive.utils import *
import time
import math
import traceback

class OdrivePositionControl():
    def __init__(self):
        self.armMsg = Float64MultiArray()
        self.armMsg.data = 5*[0]
        print("finding an odrive...")
        self.R1 = odrive.find_any(serial_number="20643681424D")
        self.R2 = odrive.find_any(serial_number="2055367B424D")
        self.R3 = odrive.find_any(serial_number="206B368C424D")
        self.stop()        
        self.clear_error()        
        rospy.Subscriber("/leftarm/position_controller/command", Float64MultiArray, self.callback)
        print("subscribed")
        print("starting calibration...")
        self.R1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.R1.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.R2.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.R2.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.R3.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        time.sleep(0.5)
        #print(self.R1)
        #print(self.R2)
        #print(self.R3)
        #print(self.R1.axis0.current_state != AXIS_STATE_IDLE)
        #while (self.R1.axis0.current_state != AXIS_STATE_IDLE) or (self.R1.axis1.current_state != AXIS_STATE_IDLE) or (self.R2.axis0.current_state != AXIS_STATE_IDLE) or (self.R2.axis1.current_state != AXIS_STATE_IDLE) or (self.R3.axis0.current_state != AXIS_STATE_IDLE):
        #while (self.R1.axis0.current_state != AXIS_STATE_IDLE) or (self.R1.axis1.current_state != AXIS_STATE_IDLE) or (self.R2.axis0.current_state != AXIS_STATE_IDLE) or (self.R2.axis1.current_state != AXIS_STATE_IDLE):
        #    time.sleep(0.1)
        #    if self.R1.axis0.error != 0:
        #        self.logger.error("Failed calibration with axis0 error 0x%x, motor error 0x%x" % (R1.axis0.error, R1.axis0.motor.error))
        #        return False
        #    elif self.R1.axis1.error != 0:
        #        self.logger.error("Failed calibration with axis1 error 0x%x, motor error 0x%x" % (R1.axis1.error, R1.axis1.motor.error))
        #        return False
        #    elif self.R2.axis1.error != 0:
        #        self.logger.error("Failed calibration with axis1 error 0x%x, motor error 0x%x" % (R2.axis0.error, R2.axis0.motor.error))
        #        return False
        #    elif self.R2.axis1.error != 0:
        #        self.logger.error("Failed calibration with axis1 error 0x%x, motor error 0x%x" % (R2.axis1.error, R2.axis1.motor.error))
        #        return False
        #    elif self.R3.axis0.error != 0:
        #        self.logger.error("Failed calibration with axis1 error 0x%x, motor error 0x%x" % (R3.axis0.error, R3.axis0.motor.error))
        #        return False
        #    else:                
        time.sleep(15)
        self.stop()
        self.engage()
        
        time.sleep(0.1)
        #self.R1.axis0.controller.input_pos = 0
        #self.R1.axis1.controller.input_pos = 0
        #self.R2.axis0.controller.input_pos = 0
        #self.R2.axis1.controller.input_pos = 0
        #self.R3.axis0.controller.input_pos = 0
        print("Odrives Initilization Complete.")
        
    def callback(self, armMsg):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", armMsg.data)
        self.R1.axis0.controller.input_pos = armMsg.data[0]
        #time.sleep(0.1)
        self.R1.axis1.controller.input_pos = armMsg.data[1]
        time.sleep(0.1)
        self.R2.axis0.controller.input_pos = armMsg.data[2]
        #time.sleep(0.1)
        self.R2.axis1.controller.input_pos = armMsg.data[3]
        time.sleep(0.1)
        self.R3.axis0.controller.input_pos = armMsg.data[4]
        time.sleep(0.1)
        #print("position is : " + str(self.R1.axis0.controller.input_pos))
        
    def checkerr(self):
        if not self.R1:
            self.logger.error("Not connected.")
            return False
            
        if self.R1.axis0.error != 0:
            print("Failed calibration with axis0 error 0x%x, motor error 0x%x" % (self.R1.axis0.error, self.R1.axis0.motor.error))
            self.clear_error()
            return False
        elif self.R1.axis1.error != 0:
            print("Failed calibration with axis1 error 0x%x, motor error 0x%x" % (self.R1.axis1.error, self.R1.axis1.motor.error))
            self.clear_error()
            return False
        elif self.R2.axis1.error != 0:
            print("Failed calibration with axis1 error 0x%x, motor error 0x%x" % (self.R2.axis0.error, self.R2.axis0.motor.error))
            self.clear_error()
            return False
        elif self.R2.axis1.error != 0:
            print("Failed calibration with axis1 error 0x%x, motor error 0x%x" % (self.R2.axis1.error, self.R2.axis1.motor.error))
            self.clear_error()
            return False
        #elif self.R3.axis0.error != 0:
        #    self.logger.error("Failed calibration with axis1 error 0x%x, motor error 0x%x" % (R3.axis0.error, R3.axis0.motor.error))
        #    return False
        else:                
            return True
            
    def engage(self):
        if not self.R1:
            print("Not connected.")
            return False
        self.R1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.R1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.R2.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.R2.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.R3.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        
    def stop(self):
        self.R1.axis0.requested_state = AXIS_STATE_IDLE
        self.R1.axis1.requested_state = AXIS_STATE_IDLE
        self.R2.axis0.requested_state = AXIS_STATE_IDLE
        self.R2.axis1.requested_state = AXIS_STATE_IDLE
        self.R3.axis0.requested_state = AXIS_STATE_IDLE
        
    def clear_error(self):
        self.R1.clear_errors()
        self.R2.clear_errors()
        self.R3.clear_errors()
        
class ROSLogger(object):
    """Imitate a standard Python logger, but pass the messages to rospy logging.
    """
    def debug(self, msg):    rospy.logdebug(msg)  #  print(msg) #
    def info(self, msg):     rospy.loginfo(msg)   #  print(msg) #
    def warn(self, msg):     rospy.logwarn(msg)   #  print(msg) #
    def error(self, msg):    rospy.logerr(msg)    #  print(msg) #
    def critical(self, msg): rospy.logfatal(msg)  #  print(msg) #

def get_param(name, default):
    val = rospy.get_param(name, default)
    rospy.loginfo('  %s: %s', name, str(val))
    return val
    
    # Or to change a value, just assign to the property
    #R1_odrv.axis0.controller.input_pos = 1
    #print("Position setpoint is " + str(R1_odrv.axis0.controller.pos_setpoint))
    
    # And this is how function calls are done:
    #for i in [1,2,3,4]:
        #print('voltage on GPIO{} is {} Volt'.format(i, R1_odrv.get_adc_voltage(i)))
def main():
    rospy.init_node('odrive_position_control')    
    opc = OdrivePositionControl()
    rospy.spin()

def lim_range(x, out_min, out_max):
    if x>out_max:
        return out_max
    elif x<out_min:
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

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


