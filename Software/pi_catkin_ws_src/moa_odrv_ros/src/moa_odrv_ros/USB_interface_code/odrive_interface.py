#!/usr/bin/env python3
import serial
from serial.serialutil import SerialException

import sys
import time
import logging
import traceback

import odrive
from odrive.enums import *
from odrive.utils import *

import fibre
#from fibre.protocol import ChannelBrokenException, ChannelDamagedException

default_logger = logging.getLogger(__name__)
default_logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

default_logger.addHandler(ch)

class ODriveFailure(Exception):
    pass

class ODriveInterfaceAPI(object):
    driver = None
    encoder_cpr = 16384
    M0 = None
    M1 = None
    connected = False
    _preroll_started = False
    _preroll_completed = False
    #engaged = False
    
    def __init__(self, logger=None, active_odrive=None):
        self.logger = logger if logger else default_logger
        
        if active_odrive: # pass in the odrv0 object from odrivetool shell to use it directly.
            self.driver = active_odrive
            self.axes = (self.driver.axis0, self.driver.axis1)
            self.M0 = self.driver.axis0 
            self.M1  = self.driver.axis1
            self.logger.info("Loaded pre-existing ODrive interface. Check index search status.")
            self.encoder_cpr = self.driver.axis0.encoder.config.cpr
            self.connected = True
            self._preroll_started = False
            self._preroll_completed = True
                
    def __del__(self):
        self.disconnect()
        
    def update_time(self, curr_time):
        # provided so simulator can update position
        pass
                    
    def connect(self, port=None, M0=0, timeout=30):
        if self.driver:
            self.logger.info("Already connected. Disconnecting and reconnecting.")
        try:
            self.driver = odrive.find_any(timeout=timeout, logger=self.logger)
            self.axes = (self.driver.axis0, self.driver.axis1)
        except:
            self.logger.error("No ODrive found. Is device powered?")
            return False
                        
        # save some parameters for easy access
        self.M0 = self.driver.axis0 if M0 == 0 else self.driver.axis1
        self.M1 = self.driver.axis1 if M0 == 0 else self.driver.axis0
        
        # check for no errors
        for axis in [self.M0, self.M1]:
            if axis.error != 0:
                error_str = "Had error on startup, rebooting. Axis error 0x%x, motor error 0x%x, encoder error 0x%x. Rebooting." % (axis.error, axis.motor.error, axis.encoder.error)
                self.logger.error(error_str)
                self.reboot()
                return False
        
        self.encoder_cpr = self.driver.axis0.encoder.config.cpr
        
        self.connected = True
        self.logger.info("Connected to ODrive. " + self.get_version_string())
        
        self._preroll_started = False
        self._preroll_completed = False
        
        return True
        
    def disconnect(self):
        self.connected = False
        self.M0 = None
        self.M1 = None
        
        #self.engaged = False
        
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        
        try:
            self.release()
        except:
            self.logger.error("Error in timer: " + traceback.format_exc())
            return False
        finally:
            self.driver = None
        return True
        
    def get_version_string(self):
        if not self.driver or not self.connected:
            return "Not connected."
        return "ODrive %s, hw v%d.%d-%d, fw v%d.%d.%d%s, sdk v%s" % (str(self.driver.serial_number),
            self.driver.hw_version_major, self.driver.hw_version_minor, self.driver.hw_version_variant,
            self.driver.fw_version_major, self.driver.fw_version_minor, self.driver.fw_version_revision,
            "-dev" if self.driver.fw_version_unreleased else "",
            odrive.version.get_version_str())
        
        
    def reboot(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        try:
            self.driver.reboot()
        except KeyError:
            self.logger.error("Rebooted ODrive.")
        except:
            self.logger.error("Failed to reboot: " + traceback.format_exc())
        finally:
            self.driver = None
        return True
        
    def calibrate(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        
        self.logger.info("Vbus %.2fV" % self.driver.vbus_voltage)
        #for i, axis in enumerate(self.axes):
        #self.logger.info("Calibrating axis %d..." % i)
        self.logger.info("Calibrating axis BOTH axis")
        self.axes[0].requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.axes[1].requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        time.sleep(1)
        while (self.axes[0].current_state != AXIS_STATE_IDLE)|(self.axes[1].current_state != AXIS_STATE_IDLE):
            time.sleep(0.1)
        if self.axes[0].error != 0:
            self.logger.error("Failed calibration with axis0 error 0x%x, motor error 0x%x" % (self.axes[0].error, self.axes[0].motor.error))
            return False
        elif self.axes[1].error != 0:
            self.logger.error("Failed calibration with axis1 error 0x%x, motor error 0x%x" % (self.axes[1].error, self.axes[1].motor.error))
            return False
        else:                
            return True        
    
    def engaged(self):
        if self.driver and hasattr(self, 'axes'):
            return self.axes[0].current_state == AXIS_STATE_CLOSED_LOOP_CONTROL and self.axes[1].current_state == AXIS_STATE_CLOSED_LOOP_CONTROL
        else:
            return False
    
    def idle(self):
        if self.driver and hasattr(self, 'axes'):
            return self.axes[0].current_state == AXIS_STATE_IDLE and self.axes[1].current_state == AXIS_STATE_IDLE
        else:
            return False
        
    def engage(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False

        #self.logger.debug("Setting drive mode.")
        for axis in self.axes:
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            #axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        #self.engaged = True
        return True
        
    def release(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        #self.logger.debug("Releasing.")
        for axis in self.axes: 
            axis.requested_state = AXIS_STATE_IDLE

        #self.engaged = False
        return True
    
    def drive(self, M0_motor_val, M1_motor_val):
        if not self.driver:
            self.logger.error("Not connected.")
            return
        self.M0.controller.input_pos = M0_motor_val
        self.M1.controller.input_pos = M1_motor_val
    
    def feed_watchdog(self):
        self.M1.watchdog_feed()
        self.M0.watchdog_feed()
        
    def get_errors(self, clear=True):
        # TODO: add error parsing, see: https://github.com/madcowswe/ODrive/blob/master/tools/odrive/utils.py#L34
        if not self.driver:
            return None
            
        axis_error = self.axes[0].error or self.axes[1].error
        
        if axis_error:
            error_string = "Errors(hex): L: a%x m%x e%x c%x, R: a%x m%x e%x c%x" % (
                self.M1.error,  self.M1.motor.error,  self.M1.encoder.error,  self.M1.controller.error,
                self.M0.error, self.M0.motor.error, self.M0.encoder.error, self.M0.controller.error,
            )
        
        if clear:
            for axis in self.axes:
                axis.error = 0
                axis.motor.error = 0
                axis.encoder.error = 0
                axis.controller.error = 0
        
        if axis_error:
            return error_string
            
    def M1_vel_estimate(self):  return self.M1.encoder.vel_estimate   if self.M1  else 0 # units: encoder counts/s
    def M0_vel_estimate(self): return self.M0.encoder.vel_estimate  if self.M0 else 0 # neg is forward for M0
    def M1_pos(self):           return self.M1.encoder.pos_cpr        if self.M1  else 0  # units: encoder counts
    def M0_pos(self):          return self.M0.encoder.pos_cpr       if self.M0 else 0   # sign!
    
    # TODO check these match the M0 motors, but it doesn't matter for now
    def M1_temperature(self):   return self.M1.motor.get_inverter_temp()  if self.M1  else 0.
    def M0_temperature(self):  return self.M0.motor.get_inverter_temp() if self.M0 else 0.
    
    def M1_current(self):       return self.M1.motor.current_control.Ibus  if self.M1 and self.M1.current_state > 1 else 0.
    def M0_current(self):      return self.M0.motor.current_control.Ibus if self.M0 and self.M0.current_state > 1 else 0.
    
    # from axis.hpp: https://github.com/madcowswe/ODrive/blob/767a2762f9b294b687d761029ef39e742bdf4539/Firmware/MotorControl/axis.hpp#L26
    MOTOR_STATES = [
        "UNDEFINED",                  #<! will fall through to idle
        "IDLE",                       #<! disable PWM and do nothing
        "STARTUP_SEQUENCE",           #<! the actual sequence is defined by the config.startup_... flags
        "FULL_CALIBRATION_SEQUENCE",  #<! run all calibration procedures, then idle
        "MOTOR_CALIBRATION",          #//<! run motor calibration
        "SENSORLESS_CONTROL",         #//<! run sensorless control
        "ENCODER_INDEX_SEARCH",       #//<! run encoder index search
        "ENCODER_OFFSET_CALIBRATION", #//<! run encoder offset calibration
        "CLOSED_LOOP_CONTROL",        #//<! run closed loop control
        "LOCKIN_SPIN",                #//<! run lockin spin
        "ENCODER_DIR_FIND",
        ]
        
    def M1_state(self):       return self.MOTOR_STATES[self.M1.current_state] if self.M1 else "NOT_CONNECTED"
    def M0_state(self):      return self.MOTOR_STATES[self.M0.current_state] if self.M0 else "NOT_CONNECTED"
    
    def bus_voltage(self):      return self.driver.vbus_voltage if self.M1 else 0.
    
