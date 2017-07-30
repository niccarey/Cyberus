#!/usr/bin/env python
# -*- coding: utf-8 -*-
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

# upon initialization, MotorStatus takes a motor hat instance and an id number
class MotorStatus:
    
    def __init__(self, mhInstance, ident):
        self.ident = ident
        self.current_speed = 0

    def simple_lowpass(self, v_current, v_des):
        # low pass filter to mitigate discontinuities in motor commands
        v_new = int(round(c0*v_current + c1*(v_des - v_current)))
        return v_new


    def motorControl(self, des_speed):
        # bring motor speed from current to desired
        # use a low pass filter to eliminate discontinuities
        
        if abs(des_speed - self.current_speed) < thresh:
            # if we are close enough, don't need the filter
            v_new = self.des_speed
        
        else:
            v_new = self.simple_lowpass(self.current_speed, des_speed)
        
        self.current_speed = v_new

        mot = mhInstance.getMotor(self.ident)
        if self.current_speed < 0:
            mot.setSpeed(-1 * self.current_speed)
            mot.run(Adafruit_MotorHAT.BACKWARD)
        else:
            mot.setSpeed(self.current_speed)
            mot.run(Adafruit_MotorHAT.FORWARD)


    # shut down motors
    def turnOffMotor(self):
        mh.getMotor(self.ident).run(Adafruit_MotorHAT.RELEASE)
