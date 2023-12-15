# -*- coding: utf-8 -*-
"""
Created on Mon Oct 23 14:33:23 2023

@author: tfein
"""

import pyb
from Romi_Motor import L6206
from Encoder import Encoder
from ClosedLoop import PIDController
from pyb import repl_uart, UART
import task_share
import time
import cotask
from BNO055 import BNO055

# UART setup
repl_uart(None)
ser_uart = UART(2, 115200 )
ser = pyb.USB_VCP()
ser.write(b'Hello World! \r\n')

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

# Create the BNO055 sensor instance with the specified pins
sda_pin = 'B8'  # SDA pin number
scl_pin = 'C9'  # SCL pin number
rst_pin = 'C2'  # RST pin number
sensor = BNO055(sda_pin, scl_pin, rst_pin)
sensor.remap_axes()

command_buffer = ""

collected_data_queue = task_share.Queue('H', 500, name="Collected Data Queue")
data_collection_done = task_share.Share(type_code='B', name="Data Collection Flag")

open_loop_mode_A = True
open_loop_mode_B = True
#duty = 0


def start_motors():
    pid_A.setpoint = 20
    pid_B.setpoint = 20
    ser.write(b"\r\nMotors started!\r\n")

def stop_motors():
    pid_A.setpoint = 0
    pid_B.setpoint = 0
    ser.write(b"\r\nMotors stopped!\r\n")

def get_motor_status():
    ser.write(f"\r\nPosition A: {encoder_A.get_delta()}\r\n".encode())
    ser.write(f"Position B: {encoder_B.get_delta()}\r\n".encode())

def set_motor_speed(speed):
    pid_A.setpoint = speed
    pid_B.setpoint = -speed
    ser.write(f"\r\nMotor speed set to: {speed}!\r\n".encode())

def print_euler(sensor):
    euler_angles = sensor.read_euler_angles()
    print("Euler Angles:", euler_angles)
    #ser.write(b"\r\nEncoder A zeroed!\r\n")
    
def print_angular(sensor):
    angular_velocity = sensor.read_angular_velocity()
    print("Angular Velocity:", angular_velocity)

def calibrate_sensor(sensor):
    # Load existing calibration or calibrate
    sensor.load_calibration()

    # Read and print the calibration status
    #calibration_status = sensor.check_calibration_status()
    #print("Calibration Status:", calibration_status)

    # Read and print the calibration coefficients
    calibration_coefficients = sensor.get_calibration_coefficients()
    print("Calibration Coefficients:", calibration_coefficients)

    # Save the calibration coefficients to a file
    with open('IMU_cal_coeffs.txt', 'w') as file:
        formatted_coeffs = ','.join('0x{:02x}'.format(coeff) for coeff in calibration_coefficients)
        file.write(formatted_coeffs)
        print("Calibration coefficients saved to 'IMU_cal_coeffs.txt'")

def pivot_to_north(sensor):
    target_heading = 0    # Target heading in degrees (North)
    tolerance = 200       # Tolerance in degrees

    while True:
        heading, roll, pitch = sensor.read_euler_angles()  # Get current heading
        heading_error = heading - target_heading

        # Check if within tolerance
        if abs(heading_error) <= tolerance:
            pid_A.setpoint = 0
            pid_B.setpoint = 0
            mot_A.set_duty(0)
            mot_B.set_duty(0)
            break

        # Determine rotation direction
        if heading_error < 0 or heading_error > 180:
            # Rotate clockwise
            mot_A.set_duty(8)
            mot_B.set_duty(-8)
        else:
            # Rotate counterclockwise
            mot_A.set_duty(-8)
            mot_B.set_duty(8)

        pyb.delay(5)  # Short delay to prevent too fast rotation
    
    
def zero_encoder(encoder):
    if encoder == 'A':
        encoder_A.zero()
        ser.write(b"\r\nEncoder A zeroed!\r\n")
    elif encoder == 'B':
        encoder_B.zero()
        ser.write(b"\r\nEncoder B zeroed!\r\n")

def get_encoder_position(encoder):
    if encoder == 'A':
        position = encoder_A.get_position()
        ser.write(f"\r\nPosition A: {position} counts\r\n".encode())
    elif encoder == 'B':
        position = encoder_B.get_position()
        ser.write(f"\r\nPosition B: {position} counts\r\n".encode())
    else:
        ser.write(b"\r\nUnknown encoder!\r\n")

        
def get_encoder_delta(encoder):
    if encoder == 'A':
        delta = encoder_A.get_delta()
        ser.write(f"\r\nDelta A: {delta}\r\n".encode())
    elif encoder == 'B':
        delta = encoder_B.get_delta()
        ser.write(f"\r\nDelta B: {delta}\r\n".encode())
    else:
        ser.write(b"\r\nUnknown encoder!\r\n")
        
def get_encoder_velocity(encoder):
    if encoder == 'A':
        velocity = encoder_A.get_velocity()
        ser.write(f"\r\nVelocity A: {velocity}\r\n".encode())
    elif encoder == 'B':
        velocity = encoder_B.get_velocity()
        ser.write(f"\r\nVelocity B: {velocity}\r\n".encode())
    else:
        ser.write(b"\r\nUnknown encoder!\r\n")

def position_data(encoder_label):
    end_time = time.time() + 30
    data_points = []
    ser.write(b"\r\nCollecting Data for 30 Seconds!\r\n")
    while time.time() < end_time:
        if encoder_label == 'A':
            encoder_A.update()
            speed = encoder_A.get_velocity()
            position = encoder_A.get_position()  # Assuming get_position() is a method. Adjust as needed.
        else:
            encoder_B.update()
            speed = encoder_B.get_velocity()
            position = encoder_B.get_position()
        
        data_points.append((speed, position))
        time.sleep(0.1)  # Collect data every 100ms. Adjust as needed.
    
    return data_points

        
def set_motor_duty_cycle(encoder):
    #global duty
    ser.write(b"\r\nEnter duty cycle (-100 to 100):")
    duty_buffer = ""
    
    while True:
        if ser.any():
            incoming_data = ser.read(ser.any()).decode()
            for char in incoming_data:
                ser.write(char.encode())  # Echo back
                if char not in ['\r', '\n']:
                    duty_buffer += char
                else:
                    try:
                        duty = float(duty_buffer)
                        if -100 <= duty <= 100:
                            if encoder == 'A':
                                mot_A.set_duty(duty)
                                ser.write(f"\r\nDuty cycle for motor A set to: {duty}%!\r\n".encode())
                            else:
                                mot_B.set_duty(duty)
                                ser.write(f"\r\nDuty cycle for motor B set to: {duty}%!\r\n".encode())
                        else:
                            ser.write(b"\r\nDuty cycle out of bounds! Please enter a value between -100 and 100.\r\n")
                    except ValueError:
                        ser.write(b"\r\nInvalid duty cycle value!\r\n")
                    return

def switch_to_closed_loop(encoder):
    global open_loop_mode_A, open_loop_mode_B

    if encoder == 'A':
        open_loop_mode_A = False
        ser.write(b"\r\nMotor A switched to closed-loop mode!\r\n")
    else:
        open_loop_mode_B = False
        ser.write(b"\r\nMotor B switched to closed-loop mode!\r\n")
        
def switch_to_open_loop(encoder):
    global open_loop_mode_A, open_loop_mode_B

    if encoder == 'A':
        open_loop_mode_A = True
        ser.write(b"\r\nMotor A switched to open-loop mode!\r\n")
    else:
        open_loop_mode_B = True
        ser.write(b"\r\nMotor B switched to open-loop mode!\r\n")
        
#Closed Loop Tasks
def set_pid_gains(encoder, kp, ki, kd):
    if encoder == 'A':
        pid_A.kp = kp
        pid_A.ki = ki
        pid_A.kd = kd
        ser.write(b"\r\nMotor A PID gains set!\r\n")
    else:
        pid_B.kp = kp
        pid_B.ki = ki
        pid_B.kd = kd
        ser.write(b"\r\nMotor B PID gains set!\r\n")

def read_uart_line():  #Used in set_pid_gains
    line = []
    while True:
        if ser.any():
            char = ser.read(1)
            ser.write(char)  # Echo the character
            if char == b'\r' or char == b'\n':
                break
            line.append(char)
    return b''.join(line).decode().strip()

def set_velocity_setpoint(motor, setpoint):
    if motor == 'A':
        pid_A.setpoint = setpoint
        ser.write(f"\r\nVelocity setpoint for Motor A set to: {setpoint}\r\n".encode())
    elif motor == 'B':
        pid_B.setpoint = setpoint
        ser.write(f"\r\nVelocity setpoint for Motor B set to: {setpoint}\r\n".encode())
    else:
        ser.write(b"\r\nInvalid motor choice!\r\n")
        
def trigger_step_response(encoder, duration=300, sample_rate=1):
    # 1. Set initial setpoint to 0
    if encoder == 'A':
        #print("1")
        pid_A.setpoint = 0
    else:
        #print("2")
        pid_B.setpoint = 0
        
    #print("3")
      # Allow for initial conditions to stabilize if needed
    #print("4")
    # 2. Set the new setpoint
    if encoder == 'A':
        #print("5")
        pid_A.setpoint = 20
        #print("5.5")
    else:
        #print("6")
        pid_B.setpoint = 20
        #print("6.5")

    # 3. Start collecting data
    data_points = []
    #print("7")
    start_time = pyb.millis()
    #print("8")

    while pyb.elapsed_millis(start_time) < duration:
        #print(f"{start_time}")
        #encoder_A.update()  # Update the encoder reading
        #encoder_B.update()  # Update the encoder reading
        #print("10")
        if encoder == 'A':
            Motor_Driver_A()
            velocity = encoder_A.get_velocity()
            #print("11")
            data_points.append(velocity)
            #print("12")
        else:
            Motor_Driver_B()
            velocity = encoder_B.get_velocity()
            #print("13")
            data_points.append(velocity)
            #print("14")
        
        pyb.delay(sample_rate)
        #print("15")
    
    return data_points

'''def trigger_step_response(encoder, duration=3000, sample_rate=10):
    # [initial setup similar to previous]
    
    start_time = pyb.millis()
    while pyb.elapsed_millis(start_time) < duration:
        if encoder == 'A':
            pid_A.setpoint = 20
            encoder_A.update()
            velocity = encoder_A.get_velocity()
            collected_data_queue.put(velocity)  # Insert velocity data into the queue
        else:
            pid_B.setpoint = 20
            encoder_B.update()
            velocity = encoder_B.get_velocity()
            collected_data_queue.put(velocity)
        
        pyb.delay(sample_rate)

    data_collection_done.put(True)  # Signal that data collection is done
    yield

def print_collected_data():
    while not data_collection_done.get():  # Wait till data collection is done
        yield
    
    while collected_data_queue.any():
        velocity = collected_data_queue.get()
        ser.write("Motor Velocity: {}\r\n".format(velocity).encode())
        yield'''

#new task to send data to pc
def send_data(encoder, data_points):
    for point in data_points:
        if encoder == 'A':
            ser_uart.write("{}\r\n".format(point).encode())   # Motor A Velocity
        else:
            ser_uart.write("{}\r\n".format(point).encode())   # Motor B Velocity
        time.sleep(0.01)  # Small delay between writes, adjust as necessary.
        
def send_data_open(encoder, data_points):
    for point in data_points:
        velocity, position = point
        if encoder == 'A':
            ser_uart.write("{}\t{}\r\n".format(velocity, position).encode())
        else:
            ser_uart.write("{}\t{}\r\n".format(velocity, position).encode())
        time.sleep(0.01)  # Small delay between writes, adjust as necessary.



def diagnostic_step_response(encoder, duration=3000, sample_rate=10):
    data_points = []
    start_time = pyb.millis()
    
    while pyb.elapsed_millis(start_time) < duration:
        if encoder == 'A':
            encoder_A.update()
            velocity = encoder_A.get_velocity()
        else:
            encoder_B.update()
            velocity = encoder_B.get_velocity()
        
        data_points.append(velocity)
        pyb.delay(sample_rate)
    
    # After collecting, print all at once (outside of the loop)
    for val in data_points:
        print(val)

def Motor_Driver_Test():
    # Update encoder values
    encoder_A.update()
    encoder_B.update()
    
    # Read encoder values
    position_A = encoder_A.get_velocity()
    position_B = encoder_B.get_velocity()
    
    # Get control actions from PID controllers
    control_A = pid_A.update(position_A)
    control_B = pid_B.update(position_B)
    
    control_A = max(min(control_A, 100), -100)  #Limit Controls to min and max duty cycle
    control_B = max(min(control_B, 100), -100)

    #print("Position A:", position_A)
    #print("Position B:", position_B)
    
    # Apply control actions to motors
    mot_A.set_duty(control_A)
    mot_B.set_duty(control_B)
    
def Motor_Driver_A():
    if open_loop_mode_A == True:
        #mot_A.set_duty(duty)
        encoder_A.update()
        
    else:    
        # Update encoder values
        encoder_A.update()
    
        # Read encoder values
        position_A = encoder_A.get_velocity()
    
        # Get control actions from PID controllers
        control_A = pid_A.update(position_A)
        control_A = max(min(control_A, 100), -100)  #Limit Controls to min and max duty cycle
    
        # Apply control actions to motors
        mot_A.set_duty(control_A)
    
def Motor_Driver_B():
    if open_loop_mode_A == True:
        #mot_A.set_duty(duty)
        encoder_B.update()
        
    else:
        # Update encoder values
        encoder_B.update()
    
        # Read encoder values
        position_B = encoder_B.get_velocity()
    
        # Get control actions from PID controllers
        control_B = pid_B.update(position_B)
        control_B = max(min(control_B, 100), -100)
    
        # Apply control actions to motors
        mot_B.set_duty(control_B)

def Command_Check():
    command_buffer = ""
    while True:
        if ser.any():
            incoming_data = ser.read(ser.any()).decode()
            for char in incoming_data:
                ser.write(char.encode())
                if char not in ['\r', '\n']:
                    command_buffer += char
                else:
                    if command_buffer:
                        if command_buffer == "test":
                            ser.write(b"\r\nEcho test successful!\r\n")
                        elif command_buffer == "start":
                            start_motors()
                        elif command_buffer == "stop":
                            stop_motors()
                        elif command_buffer == "status":
                            get_motor_status()
                        elif command_buffer.startswith("set_speed "):
                            try:
                                speed = float(command_buffer.split()[1])
                                set_motor_speed(speed)
                            except (IndexError, ValueError):
                                ser.write(b"\r\nInvalid speed value!\r\n")
                        
                                
                        elif command_buffer == "e":
                            print_euler(sensor)
                        elif command_buffer == "a":
                            print_angular(sensor)
                        elif command_buffer == "calibrate":
                            calibrate_sensor(sensor)
                        elif command_buffer == "n":
                            print_euler(sensor)

                        
                        
                        
                        #Open Loop Commands
                        #Zero Encoders
                        elif command_buffer == "z":
                            zero_encoder('A')
                        elif command_buffer == "Z":
                            zero_encoder('B')
                        #Print Position For Encoders
                        elif command_buffer == "p":
                            get_encoder_position('A')
                        elif command_buffer == "P":
                            get_encoder_position('B')
                        #Print Delta For Encoders
                        elif command_buffer == "d":
                            get_encoder_delta('A')        #Delta is in ticks/ms
                        elif command_buffer == "D":
                            get_encoder_delta('B')
                        #Print Velocity For Encoders    
                        elif command_buffer == "v":
                             get_encoder_velocity('A')    #Velocity is in rad/s
                        elif command_buffer == "V":
                            get_encoder_velocity('B')
                        #Enter Duty Cycle For Motors
                        elif command_buffer == "m":
                            set_motor_duty_cycle('A')
                        elif command_buffer == "M":
                            set_motor_duty_cycle('B')
                        #Add Collect Speed and Data to graph here
                        
                        #Switch to Closed Loop mode
                        elif command_buffer == "c":
                            switch_to_closed_loop('A')
                        elif command_buffer == "C":
                            switch_to_closed_loop('B')
                            
                        #Closed Loop Commands
                        #Set the PID Gains, 0 must be 0.0
                        elif command_buffer == "k":
                            ser.write(b"\r\nEnter PID gains for Motor A as: kp ki kd\r\n")
                            gains = read_uart_line().split()
                            try:
                                kp, ki, kd = map(float, gains)
                                set_pid_gains('A', kp, ki, kd)
                            except ValueError:
                                ser.write(b"\r\nInvalid gains!\r\n")

                        elif command_buffer == "K":
                            ser.write(b"\r\nEnter PID gains for Motor B as: kp ki kd\r\n")
                            gains = read_uart_line().split()
                            try:
                                kp, ki, kd = map(float, gains)
                                set_pid_gains('B', kp, ki, kd)
                            except ValueError:
                                ser.write(b"\r\nInvalid gains!\r\n")
                                
                        #Set the Setpoint for Motors
                        elif command_buffer == "s":
                            ser.write(b"\r\nEnter velocity setpoint for Motor A:\r\n")
                            try:
                                setpoint = float(read_uart_line())
                                set_velocity_setpoint('A', setpoint)
                            except ValueError:
                                ser.write(b"\r\nInvalid setpoint value!\r\n")

                        elif command_buffer == "S":
                            ser.write(b"\r\nEnter velocity setpoint for Motor B:\r\n")
                            try:
                                setpoint = float(read_uart_line())
                                set_velocity_setpoint('B', setpoint)
                            except ValueError:
                                ser.write(b"\r\nInvalid setpoint value!\r\n")
                            
                        #Trigger Step Response and send data to PC
                        elif command_buffer == 'r':
                            print("Triggering motor A")  # For debugging
                            data_points = trigger_step_response('A')  # For Motor A as an example
                            send_data('A', data_points)
                            #trigger_step_response('A')
                            #print_collected_data()
                        elif command_buffer == 'R':
                            print("Triggering motor B")  # For debugging
                            
                            data_points = trigger_step_response('B')  # For Motor A as an example
                            send_data('B', data_points)
                            #trigger_step_response('B')
                            #print_collected_data()
                        elif command_buffer == "o":
                            switch_to_open_loop('A')
                        elif command_buffer == "O":
                            switch_to_open_loop('B')


       
                            
                            
                            

                        else:
                            ser.write(b"\r\nUnknown command!\r\n")
                        command_buffer = ""
                        yield
                    yield
                yield
            yield
    
task_motor_A = cotask.Task(Motor_Driver_A, name='Motor A Driver', priority=2, period=10)  # Example period of 10ms
task_motor_B = cotask.Task(Motor_Driver_B, name='Motor B Driver', priority=2, period=10)
task_command_check = cotask.Task(Command_Check, name='Command Check', priority=1, period=3)  # Lower priority, 100ms

# Add these tasks to your task list
cotask.task_list.append(task_motor_A)
cotask.task_list.append(task_motor_B)
cotask.task_list.append(task_command_check)        


while True:
    
    Motor_Driver_A()
    Motor_Driver_B()

    # Check for UART command
    if ser.any():
        incoming_data = ser.read(ser.any()).decode()
        for char in incoming_data:
            ser.write(char.encode())
            if char not in ['\r', '\n']:
                command_buffer += char
            else:
                if command_buffer:
                    if command_buffer == "test":
                        ser.write(b"\r\nEcho test successful!\r\n")
                    elif command_buffer == "start":
                        start_motors()
                    elif command_buffer == "stop":
                        stop_motors()
                    elif command_buffer == "status":
                        get_motor_status()
                    elif command_buffer.startswith("set_speed "):
                        try:
                            speed = float(command_buffer.split()[1])
                            set_motor_speed(speed)
                        except (IndexError, ValueError):
                            ser.write(b"\r\nInvalid speed value!\r\n")
                    
                    elif command_buffer == "e":
                        print_euler(sensor)
                    elif command_buffer == "n":
                        pivot_to_north(sensor)
                    elif command_buffer == "a":
                        print_angular(sensor)
                    elif command_buffer == "calibrate":
                        calibrate_sensor(sensor)                  
                    
                    #Open Loop Commands
                    #Zero Encoders
                    elif command_buffer == "z":
                        zero_encoder('A')
                    elif command_buffer == "Z":
                        zero_encoder('B')
                    #Print Position For Encoders
                    elif command_buffer == "p":
                        get_encoder_position('A')
                    elif command_buffer == "P":
                        get_encoder_position('B')
                    #Print Delta For Encoders
                    elif command_buffer == "d":
                        get_encoder_delta('A')        #Delta is in ticks/ms
                    elif command_buffer == "D":
                        get_encoder_delta('B')
                    #Print Velocity For Encoders    
                    elif command_buffer == "v":
                         get_encoder_velocity('A')    #Velocity is in rad/s
                    elif command_buffer == "V":
                        get_encoder_velocity('B')
                    #Enter Duty Cycle For Motors
                    elif command_buffer == "m":
                        set_motor_duty_cycle('A')
                    elif command_buffer == "M":
                        set_motor_duty_cycle('B')
                    #Add Collect Speed and Data to graph here
                    elif command_buffer == "g":
                        data = position_data('A')
                        send_data_open('A', data)  # Assuming you have a function to send data to the PC. Adjust as needed.

                    elif command_buffer == "G":
                        data = position_data('B')
                        send_data_open('B', data)

                    #Switch to Closed Loop mode
                    elif command_buffer == "c":
                        switch_to_closed_loop('A')
                    elif command_buffer == "C":
                        switch_to_closed_loop('B')
                        
                    #Closed Loop Commands
                    #Set the PID Gains, 0 must be 0.0
                    elif command_buffer == "k":
                        ser.write(b"\r\nEnter PID gains for Motor A as: kp ki kd\r\n")
                        gains = read_uart_line().split()
                        try:
                            kp, ki, kd = map(float, gains)
                            set_pid_gains('A', kp, ki, kd)
                        except ValueError:
                            ser.write(b"\r\nInvalid gains!\r\n")

                    elif command_buffer == "K":
                        ser.write(b"\r\nEnter PID gains for Motor B as: kp ki kd\r\n")
                        gains = read_uart_line().split()
                        try:
                            kp, ki, kd = map(float, gains)
                            set_pid_gains('B', kp, ki, kd)
                        except ValueError:
                            ser.write(b"\r\nInvalid gains!\r\n")
                            
                    #Set the Setpoint for Motors
                    elif command_buffer == "s":
                        ser.write(b"\r\nEnter velocity setpoint for Motor A:\r\n")
                        try:
                            setpoint = float(read_uart_line())
                            set_velocity_setpoint('A', setpoint)
                        except ValueError:
                            ser.write(b"\r\nInvalid setpoint value!\r\n")

                    elif command_buffer == "S":
                        ser.write(b"\r\nEnter velocity setpoint for Motor B:\r\n")
                        try:
                            setpoint = float(read_uart_line())
                            set_velocity_setpoint('B', setpoint)
                        except ValueError:
                            ser.write(b"\r\nInvalid setpoint value!\r\n")
                        
                    #Trigger Step Response and send data to PC
                    elif command_buffer == 'r':
                        print("Triggering motor A")  # For debugging
                        data_points = trigger_step_response('A')  # For Motor A as an example
                        send_data('A', data_points)
                        #trigger_step_response('A')
                        #print_collected_data()
                    elif command_buffer == 'R':
                        print("Triggering motor B")  # For debugging
                        
                        data_points = trigger_step_response('B')  # For Motor A as an example
                        send_data('B', data_points)
                        #trigger_step_response('B')
                        #print_collected_data()
                    elif command_buffer == "o":
                        switch_to_open_loop('A')
                    elif command_buffer == "O":
                        switch_to_open_loop('B')


   
                        
                        
                        

                    else:
                        ser.write(b"\r\nUnknown command!\r\n")
                    command_buffer = ""



    pyb.delay(1)
