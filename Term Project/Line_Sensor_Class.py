# -*- coding: utf-8 -*-
"""
Created on Tue Nov 28 14:07:07 2023

@author: tfein
"""

import pyb

class Line_Sensor:
    '''!@brief Interface with Line Sensor on the Romi.
    @details This class provides methods to interface with a Polulu-qtrmd 07a, which allows us to continously detect analog values from 7 indiviual sensors on an array.
    '''
    def __init__(self):
        """Initialize the Line Sensor with the recquired pins, and create ADC objects for each of the 7 sensors"""
        # Define the analog pins connected to the sensors
        self.sensor_Pin1 = pyb.Pin.cpu.C0  # Left most pin
        self.sensor_Pin2 = pyb.Pin.cpu.C1
        self.sensor_Pin3 = pyb.Pin.cpu.B0
        self.sensor_Pin4 = pyb.Pin.cpu.A4  # Middle Pin
        self.sensor_Pin5 = pyb.Pin.cpu.A1
        self.sensor_Pin6 = pyb.Pin.cpu.A0
        self.sensor_Pin7 = pyb.Pin.cpu.C3  # Right Most Pin
        
        # Create ADC objects for each sensor
        self.adc1 = pyb.ADC(self.sensor_Pin1)
        self.adc2 = pyb.ADC(self.sensor_Pin2)
        self.adc3 = pyb.ADC(self.sensor_Pin3)
        self.adc4 = pyb.ADC(self.sensor_Pin4)
        self.adc5 = pyb.ADC(self.sensor_Pin5)
        self.adc6 = pyb.ADC(self.sensor_Pin6)
        self.adc7 = pyb.ADC(self.sensor_Pin7)

    def read_values(self):
        """Read the ADC values from each of the Sensors"""
        # Read the analog values from the sensors and return them
        sensorValue1 = self.adc1.read()
        sensorValue2 = self.adc2.read()
        sensorValue3 = self.adc3.read()
        sensorValue4 = self.adc4.read()
        sensorValue5 = self.adc5.read()
        sensorValue6 = self.adc6.read()
        sensorValue7 = self.adc7.read()

        return [sensorValue1, sensorValue2, sensorValue3, sensorValue4, sensorValue5, sensorValue6, sensorValue7]

    def print_values(self):
        """Prints the ADC values from each of the Sensors"""
        # Read sensor values
        sensor_values = self.read_values()

        # Format and print the sensor values
        usb_vcp = pyb.USB_VCP()
        formatted_values = " ".join(str(val) for val in sensor_values)
        usb_vcp.send(formatted_values + "\r\n")
        
'''
# Example usage
sensor_array = ReflectanceSensorArray()
while True:
    sensor_array.print_values()
    pyb.delay(100)  # Small delay to limit the speed of the loop'''
