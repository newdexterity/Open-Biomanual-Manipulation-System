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
import math
import traceback
import can
import cantools
import time
import sys, os
__location__ = os.path.realpath(os.path.join(os.getcwd(),os.path.dirname(__file__)))

class OdrivePositionControl:
    def __init__(self):
        self.lArmMsg = Float64MultiArray()
        self.lArmMsg.data = 5 * [0]
        self.rArmMsg = Float64MultiArray()
        self.rArmMsg.data = 5 * [0]
        self.db = cantools.database.load_file(os.path.join(__location__,"odrive-cansimple.dbc"))
        self.bus = can.Bus("can0", bustype="socketcan")
        self.axisIDs = [0x1,0x2,0x3,0x4,0x5,0x7,0x8,0x9,0xA,0xB]
        self.setpoint = 0.0
        rate = rospy.Rate(200)
        #self.check_err()
        self.engageall()
        print("Odrives Initilization Complete.")

        rospy.Subscriber(
            "/leftarm/position_controller/command",
            Float64MultiArray,
            self.leftarmcallback,
            queue_size=1
        )
        rospy.Subscriber(
            "/rightarm/position_controller/command",
            Float64MultiArray,
            self.rightarmcallback,
            queue_size=1
        )
        print("subscribed")
        while True:
            self.check_err()
            time.sleep(0.01)

    def leftarmcallback(self, lArmMsg):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", lArmMsg.data)
        self.check_err()
        #Anatomic angles : 
        # flex -50/180
        # abd -45/180
        # rot -80/90
        # elb 0/145 

        #Torso bend input forward max = 36 degrees, 
        self.setInputPos(self.axisIDs[5],map_values(lArmMsg.data[0], -80.0, 36.0, -22.2, 10.0))
        #left arm flex input forward max = 216 degrees, 
        self.setInputPos(self.axisIDs[6],map_values(lArmMsg.data[1], 180.0, -90.0, -50.0, 25.0))
        #left arm abd input outward max = 90 degrees, 
        self.setInputPos(self.axisIDs[7],map_values(lArmMsg.data[2], 90.0, -35.0, -25.0, 10.0))
        #left arm rot input outward max = 90 degrees, 
        self.setInputPos(self.axisIDs[8],map_values(lArmMsg.data[3], -90.0, 90.0, -25.0, 25.0))
        #left arm elbow input retract max = 80 degrees, 
        self.setInputPos(self.axisIDs[9],map_values(lArmMsg.data[4], -90, 75.0, -15.0, 13.0))

        # self.setInputPos(self.axisIDs[5],lArmMsg.data[0])
        # self.setInputPos(self.axisIDs[6],lArmMsg.data[1])
        # self.setInputPos(self.axisIDs[7],lArmMsg.data[2])
        # self.setInputPos(self.axisIDs[8],lArmMsg.data[3])
        # self.setInputPos(self.axisIDs[9],lArmMsg.data[4])
        

    def rightarmcallback(self, rArmMsg):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", rArmMsg.data)
        self.check_err()  
        #Torso bend input forward max = 36 degrees, 
        self.setInputPos(self.axisIDs[0],map_values(rArmMsg.data[0], -80.0, 36.0, -22.2, 10.0))
        #right arm flex input forward max = 216 degrees,
        self.setInputPos(self.axisIDs[1],map_values(rArmMsg.data[1], -90.0, 180.0, -25.0, 50.0))
        #right arm abd input outward max = 90 degrees,
        self.setInputPos(self.axisIDs[2],map_values(rArmMsg.data[2], 90.0, -36.0, -25.0, 10.0))
        #right arm rot input outward max = 90 degrees,
        self.setInputPos(self.axisIDs[3],map_values(rArmMsg.data[3], -90.0, 90.0, -25.0, 25.0))
        #right arm elbow input retract max = 80 degrees, 
        self.setInputPos(self.axisIDs[4],map_values(rArmMsg.data[4], -90.0, 75.0, -7.0, 7.0))

        # self.setInputPos(self.axisIDs[0],rArmMsg.data[0])
        # self.setInputPos(self.axisIDs[1],rArmMsg.data[1])
        # self.setInputPos(self.axisIDs[2],rArmMsg.data[2])
        # self.setInputPos(self.axisIDs[3],rArmMsg.data[3])
        # self.setInputPos(self.axisIDs[4],rArmMsg.data[4])

    def engageall(self):
        data = self.db.encode_message('Set_Axis_State', {'Axis_Requested_State': 0x08})
        for num, id in enumerate(self.axisIDs):
            msg = can.Message(arbitration_id=0x07 | id << 5, is_extended_id=False, data=data)
            try:
                self.bus.send(msg)
                print("Message sent on {}".format(self.bus.channel_info))
            except can.CanError:
                print("Message NOT sent!")

    def engage(self, axisID):
        print("\nPutting axis",axisID,"into AXIS_STATE_CLOSED_LOOP_CONTROL (0x08)...")
        data = self.db.encode_message('Set_Axis_State', {'Axis_Requested_State': 0x08})
        msg = can.Message(arbitration_id=0x07 | axisID  << 5, is_extended_id=False, data=data)
        try:
            self.bus.send(msg)
            time.sleep(0.01)
            self.bus.send(msg)
            print("Message sent on {}".format(self.bus.channel_info))
        except can.CanError:
            print("Message NOT sent!")

    def disengage(self, axisID):
        #0x07 is Set Axis Requested State
        print("\nPutting axis",axisID,"into AXIS_STATE_CLOSED_LOOP_CONTROL (0x08)...")
        data = self.db.encode_message('Set_Axis_State', {'Axis_Requested_State': 0x01})
        msg = can.Message(arbitration_id=0x07 | axisID  << 5, is_extended_id=False, data=data)
        try:
            self.bus.send(msg)
            print("Message sent on {}".format(self.bus.channel_info))
        except can.CanError:
            print("Message NOT sent!")

    def check_err(self):
        # print("\nChecking for errors")
        msg = self.bus.recv()
        #print("on the bus :" + str(msg))
        for num, id in enumerate(self.axisIDs):
            if msg.arbitration_id == ((id << 5) | self.db.get_message_by_name('Heartbeat').frame_id):
                errorCode = self.db.decode_message('Heartbeat', msg.data)['Axis_Error']
                if errorCode == 0x00:
                    print("No errors")
                elif errorCode == 0x100:
                    print("Axis error!  Error code: "+str(hex(errorCode)))
                    print("attempting to restart axis")                        
                    self.clear_err(id)
                    self.engage(id)
                else:
                    print("Axis error! can ID :"+str(id) +" Error code: "+str(hex(errorCode)))
                    self.clear_err(id)
                    self.engage(id)
                #break

    def clear_err(self, axisID):
        print("\nClearing axis",axisID," of all errors (0x018)...")
        data=[0, 0, 0, 0, 0, 0, 0, 0]
        msg = can.Message(arbitration_id=0x018 | axisID  << 5, is_extended_id=False, data=data)
        try:
            self.bus.send(msg)
            time.sleep(0.01)
            self.disengage(axisID)
            print("Message sent on {}".format(self.bus.channel_info))
        except can.CanError:
            print("Message NOT sent!")

    def setInputPos(self, axisID, setpoint):
        #0x0c is Set Input Pos
        data = self.db.encode_message('Set_Input_Pos', {'Input_Pos':setpoint, 'Vel_FF':0.0, 'Torque_FF':0.0})
        msg = can.Message(arbitration_id=axisID << 5 | 0x00C, data=data, is_extended_id=False)
        print("ID : "+ str(axisID)+" goto " + str(setpoint))
        self.bus.send(msg)
        time.sleep(0.001)


    def createCanId(axis_can_id, command):
        can_id = (axis_can_id << 5) | command
        return can_id

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

def main():
    rospy.init_node("odrive_can_position_control")
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
