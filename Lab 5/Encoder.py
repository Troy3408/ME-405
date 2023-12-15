# -*- coding: utf-8 -*-
"""
Created on Tue Oct 17 12:43:30 2023

@author: tfein
"""
import pyb

class Encoder:
    '''!@brief Interface with quadrature encoders using the pyb module.
    @details This class provides methods to interface with rotary encoders using the MicroPython library for STM32-based microcontrollers.
    It is capable of updating, retrieving, and resetting the encoder position and calculating the delta between position updates.
    '''
    
    THRESHOLD = 32767  # THRESHOLD is half of period
    MAX_COUNT = 65535  # 
    
    # Conversion factor for delta ticks to rad/s
    DELTA_TO_RAD_PER_SEC = 3.661886 * 2 * 3.141592653589793 / 60

    def __init__(self, timer_num, ENCA_pin, ENCB_pin, period=65535, prescaler=0):
        self.ENCA = pyb.Pin(ENCA_pin)
        self.ENCB = pyb.Pin(ENCB_pin)
        self.tim = pyb.Timer(timer_num, period=period, prescaler=prescaler)
        self.tim.channel(1, pyb.Timer.ENC_AB, pin=self.ENCA)
        self.tim.channel(2, pyb.Timer.ENC_AB, pin=self.ENCB)
        self.prev_count = self.tim.counter()
        self.position = 0
        self.delta = 0

    def update(self):
        current_count = self.tim.counter()
        raw_delta = current_count - self.prev_count
        
        if raw_delta > Encoder.THRESHOLD:
            self.delta = raw_delta - (Encoder.MAX_COUNT + 1)
        elif raw_delta < -Encoder.THRESHOLD:
            self.delta = (Encoder.MAX_COUNT + 1) + raw_delta
        else:
            self.delta = raw_delta
            
        self.position = current_count
        self.prev_count = current_count

    def get_position(self):
        return self.position

    def get_delta(self):
        '''!@brief Gets the most recent encoder delta in rad/s.
        @details Provides the last computed delta in rad/s, which was determined in the update() method.
        @return: The most recent encoder delta in rad/s.
        '''
        return self.delta
    
    def get_velocity(self):
        '''!@brief Gets the most recent encoder delta in rad/s.
        @details Provides the last computed delta in rad/s, which was determined in the update() method.
        @return: The most recent encoder delta in rad/s.
        '''
        return self.delta * Encoder.DELTA_TO_RAD_PER_SEC

    def zero(self):
        self.tim.counter(0)
        self.prev_count = 0
        self.position = 0
        self.delta = 0

