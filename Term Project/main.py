# -*- coding: utf-8 -*-
"""
Created on Tue Nov 28 14:16:42 2023

@author: tfein
"""

import pyb
from Romi_Motor import L6206
from Encoder import Encoder
from ClosedLoop import PIDController
from BNO055 import BNO055
from Line_Sensor_Class import Line_Sensor
from range_finder import Range_Finder
import time


# Enable Motors
ENA = pyb.Pin('PA6', pyb.Pin.OUT_PP)
ENB = pyb.Pin('PC8', pyb.Pin.OUT_PP)
ENA.high()
ENB.high()

# Create Timers, Motors, and Encoder Objects
PWM_tim = pyb.Timer(8, freq=1000)  # Adjust timer and frequency as needed
PWM_tim2 = pyb.Timer(4, freq=1000)  # Adjust timer and frequency as needed

mot_A = L6206(PWM_tim, 'PC6', 'PC5', 'PC8')
mot_B = L6206(PWM_tim2, 'PB6', 'PA7', 'PA6')

mot_A.enable_motor()
mot_B.enable_motor()


encoder_B = Encoder(1, pyb.Pin.cpu.A8, pyb.Pin.cpu.A9)
encoder_A = Encoder(3, pyb.Pin.cpu.B4, pyb.Pin.cpu.B5)

# Initialize PID Controllers
pid_A = PIDController(kp=2.0, ki=0.2, kd=0.0, setpoint=0)   # Setpoint is in rad/sec, max is 27 r/s
pid_B = PIDController(kp=2.0, ki=0.2, kd=0.0, setpoint=0)  # Adjust as necessary

button = pyb.Pin('C13', pyb.Pin.IN, pyb.Pin.PULL_UP)
robot_running = False

# Create the BNO055 sensor instance with the specified pins
sda_pin = 'B8'  # SDA pin number
scl_pin = 'C9'  # SCL pin number
rst_pin = 'C2'  # RST pin number
sensor = BNO055(sda_pin, scl_pin, rst_pin)
sensor.remap_axes()
#Creat Object for the Line Sensor
LineSensor = Line_Sensor()
RangeFinder = Range_Finder()

# Initialize a global variable to track the previous state of the sensors
all_black_previously = False

last_heading = 0
last_heading_time = time.time()

def check_button():
    global robot_running
    if button.value() == 0:  # Button is pressed (active-low)
        pyb.delay(50)  # Debounce delay
        if button.value() == 0:  # Check again to confirm button press
            robot_running = not robot_running  # Toggle the state
            while button.value() == 0:
                pass  # Wait for button release
            pyb.delay(50)  # Additional debounce delay after release
            
def turn_left(sensor, mot_A, mot_B, LineSensor):
    Black_Threshold = 800  # Threshold for detecting black line
    Heading_Change_Target = 1450  # 90 degrees in BNO055 heading units (scale 0-5800)
    Heading_Error_Threshold = 50  # Acceptable error in heading

    # Step 1: Go straight until all sensors are black
    while not all(value >= Black_Threshold for value in LineSensor.read_values()):
        mot_A.set_duty(15)
        mot_B.set_duty(15)
        pyb.delay(50)

    # Stop motors before turning
    mot_A.set_duty(15) #center romi on black line
    mot_B.set_duty(15)
    pyb.delay(300)
    mot_A.set_duty(0)
    mot_B.set_duty(0)
    pyb.delay(500)  # Short delay to stabilize before turning

    # Step 2: Calculate the new heading
    current_heading, _, _ = sensor.read_euler_angles()
    new_heading = current_heading - Heading_Change_Target  # Subtract for left turn
    if new_heading < 0:
        new_heading += 5800  # Adjust if new heading is negative

    # Step 3: Rotate the robot to the new heading
    while True:
        current_heading, _, _ = sensor.read_euler_angles()
        if abs(current_heading - new_heading) <= Heading_Error_Threshold:
            break  # Target heading reached

        # Rotate left
        mot_A.set_duty(15)
        mot_B.set_duty(-15)
        pyb.delay(50)

    # Stop the motors after turning
    mot_A.set_duty(0)
    mot_B.set_duty(0)

def range_finder_turn(RangeFinder, mot_A, mot_B):
    idx = 0
    Range_Finder_Threshold = 200

    while True:
        range_values = RangeFinder.read_values()[0]  # Get the first (and only) value from the list
        #print("Range Finder Value:", range_values)  # Debugging

        if range_values > Range_Finder_Threshold and idx < 2:
            mot_A.set_duty(15)
            mot_B.set_duty(15)
            pyb.delay(10)  # Short delay
            #print("Moving Forward")  # Debugging
            
        elif range_values < Range_Finder_Threshold and idx == 0:
            pyb.delay(200)
            mot_A.set_duty(-25)
            mot_B.set_duty(25)
            pyb.delay(475)
            idx = 1
            mot_A.set_duty(15)
            mot_B.set_duty(15)
            pyb.delay(1700)
            #print("First Turn")  # Debugging
            
        elif range_values < Range_Finder_Threshold and idx == 1:
            pyb.delay(200)
            mot_A.set_duty(-25)
            mot_B.set_duty(25)
            pyb.delay(475)
            idx = 2
            mot_A.set_duty(15)
            mot_B.set_duty(15)
            pyb.delay(800)
            #mot_A.set_duty(25)
            #mot_B.set_duty(-25)
            #pyb.delay(475)
            turn_left(sensor, mot_A, mot_B, LineSensor)
            #mot_A.set_duty(15)
            #mot_B.set_duty(15)
            #print("Second Turn and Exiting")  # Debugging
            return

        

def check_heading_and_reverse(sensor, mot_A, mot_B):
    global last_heading_time, last_heading
    Heading_Change_Threshold = 50  # Minimum change in heading to be considered significant
    No_Change_Time_Threshold = 3  # 5 seconds

    current_heading, _, _ = sensor.read_euler_angles()

    
    if abs(current_heading - last_heading) < Heading_Change_Threshold and middle_black:
        if time.time() - last_heading_time > No_Change_Time_Threshold:
            
            mot_A.set_duty(-30)
            mot_B.set_duty(-30)
            pyb.delay(200)  # 1 second delay 400
            mot_A.set_duty(25)
            mot_B.set_duty(-25)
            pyb.delay(500)
            mot_A.set_duty(15)
            mot_B.set_duty(16)
            pyb.delay(200)   #remove
            range_finder_turn(RangeFinder, mot_A, mot_B)        
              
                   
            '''# Reverse motors for 1 second
            mot_A.set_duty(-30)
            mot_B.set_duty(-30)
            pyb.delay(200)  # 1 second delay 400
            mot_A.set_duty(25)
            mot_B.set_duty(-25)
            pyb.delay(550)
            mot_A.set_duty(16)
            mot_B.set_duty(26)
            pyb.delay(4000)
            mot_A.set_duty(15)
            mot_B.set_duty(15)
            pyb.delay(200)'''
           
            
            
            # Reset the heading time and heading
            last_heading_time = time.time()
            last_heading, _, _ = sensor.read_euler_angles()
    else:
        # Update the last heading and time
        last_heading = current_heading
        last_heading_time = time.time()


def turn_180(sensor, mot_A, mot_B):
    # Read the current Euler angles
    current_heading, _, _ = sensor.read_euler_angles()

    # Calculate the target heading
    target_heading = current_heading + 2900
    # Normalize if the target exceeds 5800
    if target_heading > 5800:
        target_heading -= 5800

    # Define a threshold for acceptable error
    error_threshold = 100  # Adjust as needed

    # Rotate the robot
    while True:
        # Read the current heading
        current_heading, _, _ = sensor.read_euler_angles()

        # Check if the current heading is within the threshold of the target heading
        if abs(current_heading - target_heading) <= error_threshold:
            # Stop the motors once the target heading is achieved
            mot_A.set_duty(0)
            mot_B.set_duty(0)
            break
        else:
            # Rotate the robot (adjust the duty cycle as needed)
            mot_A.set_duty(-20)  # Reverse one motor
            mot_B.set_duty(20)   # Forward the other motor

        pyb.delay(10)  # Short delay for loop iteration


def compare_values(range_values, sensorValues, sensor, mot_A, mot_B):
    global all_black_previously  # Use the global variable
    global middle_black
    Black_Threshold = 1000

    # Define indexes for specific sensors
    left_most_sensor = [0]
    left_sensors = [1]
    middle_sensor = [3]
    right_sensors = [5]
    right_most_sensor = [6]

    # Check specific sensors for black
    left_most_black = any(sensorValues[i] >= Black_Threshold for i in left_most_sensor)
    left_black = any(sensorValues[i] >= Black_Threshold for i in left_sensors)
    middle_black = any(sensorValues[i] >= Black_Threshold for i in middle_sensor)
    right_black = any(sensorValues[i] >= Black_Threshold for i in right_sensors)
    right_most_black = any(sensorValues[i] >= Black_Threshold for i in right_most_sensor)

    # Check if all sensors are detecting black or white
    all_black = all(value >= Black_Threshold for value in sensorValues)
    #all_white = all(value < Black_Threshold for value in sensorValues)
    
    check_heading_and_reverse(sensor, mot_A, mot_B)
    
    # If all sensors are black and previously they were not all black
    if all_black and not all_black_previously:
        pyb.delay(200)  # Wait for 0.25 seconds
        sensorValues = LineSensor.read_values()  # Read sensor values again
        all_white_now = all(value < Black_Threshold for value in sensorValues)

        # If all sensors are white after 0.25 seconds
        if all_white_now:
            pyb.delay(1500)
            mot_A.set_duty(0)  # Stop motors
            mot_B.set_duty(0)
            pyb.delay(5000)  # Stop for 10 seconds
            all_black_previously = False  # Reset the flag
            turn_180(sensor, mot_A, mot_B)
            return  # Exit the function to skip normal motor control logic

    # Update the previous state
    all_black_previously = all_black
    
    
    
    Black_Threshold = 1000
    # Define indexes for specific sensors
    left_most_sensor = [0]
    left_sensors = [1]
    middle_sensor = [3]
    right_sensors = [5]
    right_most_sensor = [6]

    # Check specific sensors for black
    left_most_black = any(sensorValues[i] >= Black_Threshold for i in left_most_sensor)
    left_black = any(sensorValues[i] >= Black_Threshold for i in left_sensors)
    middle_black = any(sensorValues[i] >= Black_Threshold for i in middle_sensor)
    right_black = any(sensorValues[i] >= Black_Threshold for i in right_sensors)
    right_most_black = any(sensorValues[i] >= Black_Threshold for i in right_most_sensor)

    # Count how many of the left-most, middle, and right-most sensors are detecting black
    specific_black_sensor_count = sum([left_most_black, middle_black, right_most_black])

    # Adjust motor speeds based on sensor readings
    if specific_black_sensor_count >= 2:
        mot_A.set_duty(15)  # Move forward if 2 or more specific sensors detect black
        mot_B.set_duty(16)
    elif middle_black:
        mot_A.set_duty(15)  # Drive forward
        mot_B.set_duty(16)
    elif left_black:
        mot_A.set_duty(45)  # Turn left
        mot_B.set_duty(-25)
    elif right_black:
        mot_A.set_duty(-25)   # Turn right
        mot_B.set_duty(45)
    else:
        mot_A.set_duty(15)  # Drive forward if all sensors are white
        mot_B.set_duty(16)



# Main loop
while True:
    #mot_A.set_duty(0)   # Stop if no line is detected
    #mot_B.set_duty(0)
    #range_finder_turn(RangeFinder, mot_A, mot_B)
    #sensorValues = LineSensor.read_values()
    #range_values = RangeFinder.read_values()
    #compare_values(range_values, sensorValues, sensor, mot_A, mot_B)
    #pyb.delay(1)
    check_button()  # Check the button state

    if robot_running:
        # Place your robot control code here
        # For example:
        sensorValues = LineSensor.read_values()
        range_values = RangeFinder.read_values()
        compare_values(range_values, sensorValues, sensor, mot_A, mot_B)
    else:
        # Stop the robot or perform any required actions when not running
        mot_A.set_duty(0)
        mot_B.set_duty(0)

    pyb.delay(1)  # Small delay to prevent excessive CPU usage

'''

while True:
    sensorValue1, sensorValue2, sensorValue3, sensorValue4, sensorValue5, sensorValue6, sensorValue7 = LineSensor.read_values()
    compare_values(sensorValue1, sensorValue2, sensorValue3, sensorValue4, sensorValue5, sensorValue6, sensorValue7)
    pyb.delay(500)  # Small delay to limit the speed of the loop'''