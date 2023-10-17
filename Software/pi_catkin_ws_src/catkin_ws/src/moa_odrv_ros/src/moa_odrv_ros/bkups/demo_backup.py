#!/usr/bin/env python
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

class ODriveNode(object):
    last_speed = 0.0
    driver = None
    prerolling = False
    
    # Startup parameters
    connect_on_startup = False
    calibrate_on_startup = False
    engage_on_startup = False
    
    publish_joint_angles = True
    # Simulation mode
    # When enabled, output simulated odometry and joint angles (TODO: do joint angles anyway from ?)
    sim_mode = False
    
    def __init__(self):
        self.sim_mode             = get_param('simulation_mode', False)
        self.publish_joint_angles = get_param('publish_joint_angles', True) # if self.sim_mode else False
        self.publish_temperatures = get_param('publish_temperatures', True)
        
        self.axis_for_right = float(get_param('~axis_for_right', 0)) # if right calibrates first, this should be 0, else 1
        
        self.connect_on_startup   = get_param('~connect_on_startup', False)
        #self.calibrate_on_startup = get_param('~calibrate_on_startup', False)
        #self.engage_on_startup    = get_param('~engage_on_startup', False)
        
        self.has_preroll     = get_param('~use_preroll', True)
                
        self.publish_current = get_param('~publish_current', True)
        
        rospy.on_shutdown(self.terminate)

        rospy.Service('connect_driver',    std_srvs.srv.Trigger, self.connect_driver)
        rospy.Service('disconnect_driver', std_srvs.srv.Trigger, self.disconnect_driver)    
        rospy.Service('calibrate_motors',         std_srvs.srv.Trigger, self.calibrate_motor)
        rospy.Service('engage_motors',            std_srvs.srv.Trigger, self.engage_motor)
        rospy.Service('release_motors',           std_srvs.srv.Trigger, self.release_motor)
                
        self.status_pub = rospy.Publisher('status', std_msgs.msg.String, latch=True, queue_size=2)
        self.status = "disconnected"
        self.status_pub.publish(self.status)
        self.publish_diagnostics = True
        if self.publish_diagnostics:
            self.diagnostic_updater = diagnostic_updater.Updater()
            self.diagnostic_updater.setHardwareID("Not connected, unknown")
            self.diagnostic_updater.add("ODrive Diagnostics", self.pub_diagnostics)
        
        self.i2t_error_latch = False
        if self.publish_current:
            #self.current_loop_count = 0
            #self.left_current_accumulator  = 0.0
            #self.right_current_accumulator = 0.0
            self.current_publisher_left  = rospy.Publisher('left/current', Float64, queue_size=2)
            self.current_publisher_right = rospy.Publisher('right/current', Float64, queue_size=2)
            self.i2t_publisher_left  = rospy.Publisher('left/i2t', Float64, queue_size=2)
            self.i2t_publisher_right = rospy.Publisher('right/i2t', Float64, queue_size=2)
            
            rospy.logdebug("ODrive will publish motor currents.")
            
            self.i2t_resume_threshold  = get_param('~i2t_resume_threshold',  222)            
            self.i2t_warning_threshold = get_param('~i2t_warning_threshold', 333)
            self.i2t_error_threshold   = get_param('~i2t_error_threshold',   666)
        
        self.last_cmd_vel_time = rospy.Time.now()
                
    def main_loop(self):
        # Main control, handle startup and error handling
        # while a ROS timer will handle the high-rate (~50Hz) comms + odometry calcs
        main_rate = rospy.Rate(1) # hz
        
        while not rospy.is_shutdown():
            try:
                main_rate.sleep()
            except rospy.ROSInterruptException: # shutdown / stop ODrive??
                break;
            self.connect_driver()
            # check for errors
            if self.driver:
                try:
                    # driver connected, but fast_comms not active -> must be an error?
                    # TODO: try resetting errors and recalibrating, not just a full disconnection
                    error_string = self.driver.get_errors(clear=True)
                    if error_string:
                        rospy.logerr("Had errors, disconnecting and retrying connection.")
                        rospy.logerr(error_string)
                        self.driver.disconnect()
                        self.status = "disconnected"
                        self.status_pub.publish(self.status)
                        self.driver = None

                except (ChannelBrokenException, ChannelDamagedException, AttributeError):
                    rospy.logerr("ODrive USB connection failure in main_loop.")
                    self.status = "disconnected"
                    self.status_pub.publish(self.status)
                    self.driver = None
                except:
                    rospy.logerr("Unknown errors accessing ODrive:" + traceback.format_exc())
                    self.status = "disconnected"
                    self.status_pub.publish(self.status)
                    self.driver = None
            
            if not self.driver:
                if not self.connect_on_startup:
                    rospy.loginfo("ODrive node started, but not connected.")
                    continue
                
                if not self.connect_driver(None)[0]:
                    rospy.logerr("Failed to connect.") # TODO: can we check for timeout here?
                    continue
                    
                if self.publish_diagnostics:
                    self.diagnostic_updater.setHardwareID(self.driver.get_version_string())
            
            else:
                rospy.loginfo("waiting for ODrive:")
                pass # loop around and try again

    def terminate(self):
        if self.driver:
            self.driver.release()
    
    # ROS services
    def connect_driver(self, request):
        if self.driver:
            return (False, "Already connected.")
        
        ODriveClass = ODriveInterfaceAPI if not self.sim_mode else ODriveInterfaceSimulator
        
        self.driver = ODriveInterfaceAPI(logger=ROSLogger())
        rospy.loginfo("Connecting to ODrive...")
        if not self.driver.connect(right_axis=self.axis_for_right):
            self.driver = None
            #rospy.logerr("Failed to connect.")
            return (False, "Failed to connect.")
            
        #rospy.loginfo("ODrive connected.")
        
        # okay, connected, 
        self.m_s_to_value = self.driver.encoder_cpr/self.tyre_circumference
        
        if self.publish_odom:
            self.old_pos_l = self.driver.left_axis.encoder.pos_cpr
            self.old_pos_r = self.driver.right_axis.encoder.pos_cpr
                
        self.status = "connected"
        self.status_pub.publish(self.status)
        return (True, "ODrive connected successfully")
    
    def disconnect_driver(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        try:
            if not self.driver.disconnect():
                return (False, "Failed disconnection, but try reconnecting.")
        except:
            rospy.logerr('Error while disconnecting: {}'.format(traceback.format_exc()))
        finally:
            self.status = "disconnected"
            self.status_pub.publish(self.status_pub)
            self.driver = None
        return (True, "Disconnection success.")
    
    def calibrate_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
            
        if self.has_preroll:
            self.odometry_update_enabled = False # disable odometry updates while we preroll
            if not self.driver.preroll(wait=True):
                self.status = "preroll_fail"
                self.status_pub.publish(self.status)
                return (False, "Failed preroll.")
            
            self.status_pub.publish("ready")
            rospy.sleep(1)
            self.odometry_update_enabled = True
        else:
            if not self.driver.calibrate():
                return (False, "Failed calibration.")
                
        return (True, "Calibration success.")
    
    def engage_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        if not self.driver.has_prerolled():
            return (False, "Not prerolled.")
        if not self.driver.engage():
            return (False, "Failed to engage motor.")
        return (True, "Engage motor success.")
    
    def release_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        if not self.driver.release():
            return (False, "Failed to release motor.")
        return (True, "Release motor success.")
    
    def pub_current(self):
        self.current_publisher_left.publish(float(self.current_l))
        self.current_publisher_right.publish(float(self.current_r))
        
        now = time.time()
        
        if not hasattr(self, 'last_pub_current_time'):
            self.last_pub_current_time = now
            self.left_energy_acc = 0
            self.right_energy_acc = 0
            return
            
        # calculate and publish i2t
        dt = now - self.last_pub_current_time
        
        power = max(0, self.current_l**2 - self.i2t_current_nominal**2)
        energy = power * dt
        self.left_energy_acc *= 1 - self.i2t_update_rate * dt
        self.left_energy_acc += energy
        
        power = max(0, self.current_r**2 - self.i2t_current_nominal**2)
        energy = power * dt
        self.right_energy_acc *= 1 - self.i2t_update_rate * dt
        self.right_energy_acc += energy
        
        self.last_pub_current_time = now
        
        self.i2t_publisher_left.publish(float(self.left_energy_acc))
        self.i2t_publisher_right.publish(float(self.right_energy_acc))
        
        # stop odrive if overheated
        if self.left_energy_acc > self.i2t_error_threshold or self.right_energy_acc > self.i2t_error_threshold:
            if not self.i2t_error_latch:
                self.driver.release()
                self.status = "overheated"
                self.i2t_error_latch = True
                rospy.logerr("ODrive has exceeded i2t error threshold, ignoring drive commands. Waiting to cool down.")
        elif self.i2t_error_latch:
            if self.left_energy_acc < self.i2t_resume_threshold and self.right_energy_acc < self.i2t_resume_threshold:
                # have cooled enough now
                self.status = "ready"
                self.i2t_error_latch = False
                rospy.logerr("ODrive has cooled below i2t resume threshold, ignoring drive commands. Waiting to cool down.")
        
        
    #     current_quantizer = 5
    #
    #     self.left_current_accumulator += self.current_l
    #     self.right_current_accumulator += self.current_r
    #
    #     self.current_loop_count += 1
    #     if self.current_loop_count >= current_quantizer:
    #         self.current_publisher_left.publish(float(self.left_current_accumulator) / current_quantizer)
    #         self.current_publisher_right.publish(float(self.right_current_accumulator) / current_quantizer)
    #
    #         self.current_loop_count = 0
    #         self.left_current_accumulator = 0.0
    #         self.right_current_accumulator = 0.0

    def pub_joint_angles(self, time_now):
        jsm = self.joint_state_msg
        jsm.header.stamp = time_now
        if self.driver:
            jsm.position[0] = 2*math.pi * self.new_pos_l  / self.encoder_cpr
            jsm.position[1] = 2*math.pi * self.new_pos_r / self.encoder_cpr
            
        self.joint_state_publisher.publish(jsm)

    def pub_diagnostics(self, stat):
        stat.add("Status", self.status)
        stat.add("Motor state L", self.motor_state_l) 
        stat.add("Motor state R", self.motor_state_r)
        stat.add("FET temp L (C)", round(self.temp_v_l,1))
        stat.add("FET temp R (C)", round(self.temp_v_r,1))
        stat.add("Motor temp L (C)", "unimplemented")
        stat.add("Motor temp R (C)", "unimplemented")
        stat.add("Motor current L (A)", round(self.current_l,1))
        stat.add("Motor current R (A)", round(self.current_r,1))
        stat.add("Voltage (V)", round(self.bus_voltage,2))
        stat.add("Motor i2t L", round(self.left_energy_acc,1))
        stat.add("Motor i2t R", round(self.right_energy_acc,1))
        
        # https://github.com/ros/common_msgs/blob/jade-devel/diagnostic_msgs/msg/DiagnosticStatus.msg
        if self.status == "disconnected":
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Not connected")
        else:
            if self.i2t_error_latch:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "i2t overheated, drive ignored until cool")
            elif self.left_energy_acc > self.i2t_warning_threshold:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Left motor over i2t warning threshold")
            elif self.left_energy_acc > self.i2t_error_threshold:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Left motor over i2t error threshold")
            elif self.right_energy_acc > self.i2t_warning_threshold:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Right motor over i2t warning threshold")
            elif self.right_energy_acc > self.i2t_error_threshold:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Right motor over i2t error threshold")
            # Everything is okay:
            else:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Running")
        
def start_odrive():
    rospy.init_node('odrive')
    odrive_node = ODriveNode()
    odrive_node.main_loop()
    #rospy.spin() 

    # Find a connected ODrive (this will block until you connect one)
    print("finding an odrive...")
    #R1_odrv = odrive.find_any(serial_number="20643681424D")
    R1_odrv = odrive.find_any(serial_number="20643681424D")
    R2_odrv = odrive.find_any(serial_number="2055367B424D")
    R3_odrv = odrive.find_any(serial_number="206B368C424D")

    # Calibrate motor and wait for it to finish
    print("starting calibration...")
    R1_odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    #while R1_odrv.axis0.current_state != AXIS_STATE_IDLE:
    #    time.sleep(0.1)
    R1_odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    #while R1_odrv.axis1.current_state != AXIS_STATE_IDLE:
    #    time.sleep(0.1)
    R2_odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    #while R2_odrv.axis0.current_state != AXIS_STATE_IDLE:
    #    time.sleep(0.1)
    R2_odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    #while R2_odrv.axis1.current_state != AXIS_STATE_IDLE:
    #    time.sleep(0.1)
    R3_odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    while R3_odrv.axis0.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
        #dump_errors(R1_odrv)
        #dump_errors(R2_odrv)
        #dump_errors(R3_odrv)
    
    print("Calibration complete...")
    time.sleep(0.5)
    #dump_errors(R1_odrv,True)
    #R1_odrv.clear_errors() 
    R1_odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    R1_odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.1)
    R2_odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    R2_odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.1)
    R3_odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.1)
    # To read a value, simply read the property
    #print("Bus voltage is " + str(R1_odrv.vbus_voltage) + "V")
    
    # Or to change a value, just assign to the property
    #R1_odrv.axis0.controller.input_pos = 1
    #print("Position setpoint is " + str(R1_odrv.axis0.controller.pos_setpoint))
    
    # And this is how function calls are done:
    #for i in [1,2,3,4]:
        #print('voltage on GPIO{} is {} Volt'.format(i, R1_odrv.get_adc_voltage(i)))

    # A sine wave to test
    t0 = time.monotonic()
    while True:
        setpoint = 1.0 * math.sin((time.monotonic() - t0)*2)
        print("goto " + str(int(setpoint)))
        time.sleep(0.1)
        R1_odrv.axis0.controller.input_pos = setpoint
        print("R1.0")
        R1_odrv.axis1.controller.input_pos = setpoint
        print("R1.1")
        time.sleep(0.1)
        R2_odrv.axis0.controller.input_pos = setpoint
        print("R2.0")
        R2_odrv.axis1.controller.input_pos = setpoint
        print("R2.1")
        time.sleep(0.1)
        R3_odrv.axis0.controller.input_pos = setpoint
        print("R3.0")
        #dump_errors(R1_odrv)
        #dump_errors(R2_odrv)
        #dump_errors(R3_odrv)
        time.sleep(0.1)
        
if __name__ == '__main__':
    try:
        start_odrive()
    except rospy.ROSInterruptException:
        pass