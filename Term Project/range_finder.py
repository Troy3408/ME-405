# -*- coding: utf-8 -*-
"""
Created on Tue Dec 12 13:33:18 2023
Test
@author: tfein
"""
"""
Test 2
"""
import pyb

class Range_Finder:
    def __init__(self):
        # Define the analog pins connected to the sensors
        self.sensor_Pin1 = pyb.Pin.cpu.C4  # Left most pin
        
        
        # Create ADC objects for each sensor
        self.adc1 = pyb.ADC(self.sensor_Pin1)
       

    def read_values(self):
        # Read the analog values from the sensors and return them
        sensorValue1 = self.adc1.read()


        return [sensorValue1]

    def print_values(self):
        # Read sensor values
        sensor_values = self.read_values()

        # Format and print the sensor values
        usb_vcp = pyb.USB_VCP()
        formatted_values = " ".join(str(val) for val in sensor_values)
        usb_vcp.send(formatted_values + "\r\n")
        
# Example usage
'''sensor_array = Range_Finder()
while True:
    sensor_array.print_values()
    pyb.delay(100)  # Small delay to limit the speed of the loop'''