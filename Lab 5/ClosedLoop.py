# -*- coding: utf-8 -*-
"""
Created on Tue Oct 17 13:10:55 2023

@author: tfein
"""

class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.setpoint = setpoint
        self.previous_error = 0.0
        self.integral = 0.0
        
    def update(self, measured_value):
        """
        Calculate PID output value for the given reference setpoint and measurement.
        
        Args:
            measured_value (float): The current value of the system state.
        
        Returns:
            float: Control action to be taken.
        """
        error = self.setpoint - measured_value
        
        # Proportional term
        P = self.kp * error
        
        # Integral term
        self.integral += error
        I = self.ki * self.integral
        
        # Derivative term
        D = self.kd * (error - self.previous_error)
        
        # Remember the error for the next loop
        self.previous_error = error
        
        CombinedPID = P + I + D
        
        return CombinedPID
    
    def set_setpoint(self, setpoint):
        """
        Set the desired setpoint for the system.
        
        Args:
            setpoint (float): Desired system state.
        """
        self.setpoint = setpoint
        # Reset previous error and integral
        self.previous_error = 0.0
        self.integral = 0.0

    def set_gains(self, kp, ki, kd):
        """
        Set the gains for the PID controller.
        
        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
    def reset(self):
        """
        Reset the PID controller state.
        """
        self.previous_error = 0.0
        self.integral = 0.0
