# -*- coding: utf-8 -*-
"""
Created on Fri Oct  6 11:59:53 2023

@author: tfein
"""

'''!@file main.py
'''
from pyb import Pin, Timer

class L6206:
    def __init__(self, PWM_tim, PWM_pin, DIR_pin, EN_pin):
        ''' Initialize the motor driver with PWM, direction, and enable pins '''
        self.PWM = Pin(PWM_pin, Pin.OUT_PP)
        self.DIR = Pin(DIR_pin, Pin.OUT_PP)
        self.EN = Pin(EN_pin, Pin.OUT_PP)
        self.timer = PWM_tim
        
        # Create PWM channel for speed control
        self.PWM_channel = self.timer.channel(1, Timer.PWM, pin=self.PWM)
        self.disable_motor()  # Start with the motor disabled

    def set_duty(self, duty):
        ''' Set the PWM duty cycle for the motor.
            Positive duty for one direction, negative for the other.
        '''
        self.enable_motor()  # Ensure the motor is enabled

        if duty <= 0:
            self.DIR.high()  # Set direction
            self.PWM_channel.pulse_width_percent(abs(duty))
        else:
            self.DIR.low()   # Reverse direction
            self.PWM_channel.pulse_width_percent(abs(duty))

    def enable_motor(self):
        ''' Enable the motor driver '''
        self.EN.high()

    def disable_motor(self):
        ''' Disable the motor driver '''
        self.EN.low()


# Example usage
PWM_tim = Timer(8, freq=1000)  # Adjust timer and frequency as needed
PWM_tim2 = Timer(4, freq=1000)  # Adjust timer and frequency as needed
'''PWM_pin = 'PC6'  # Replace with your PWM pin
DIR_pin = 'PC5'  # Replace with your direction pin
EN_pin = 'PC8'  # Replace with your enable pin'''

motor_right = L6206(PWM_tim, 'PC6', 'PC5', 'PC8')
motor_left = L6206(PWM_tim2, 'PB6', 'PA7', 'PA6')
motor_right.enable_motor()
motor_left.enable_motor()
motor_right.set_duty(0)  # Spin motor at 50% speed in one direction
motor_left.set_duty(0)
#motor_driver.set_duty(-50) # Spin motor at 50% speed in the opposite direction
#motor_driver.disable_motor()  # Disable the motor

