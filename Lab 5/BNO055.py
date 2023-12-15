# -*- coding: utf-8 -*-
"""
Created on Tue Nov 14 13:31:15 2023

@author: tfein
"""
import os
import pyb
from machine import Pin, SoftI2C

class BNO055:
    def __init__(self, sda_pin, scl_pin, rst_pin=None):
        # BNO055 default I2C address
        self.addr = 0x28  # Can be 29 or 28, 28 is alternative maybe try both

        # Initialize I2C
        self.i2c = SoftI2C(sda=Pin(sda_pin), scl=Pin(scl_pin), freq=400000)

        # Handle the reset pin
        #if rst_pin is not None:
         #   self.rst = Pin(rst_pin, Pin.OUT)
          #  self.reset_sensor()
        
        # Placeholder for register addresses and values
        # Replace with actual values from the BNO055 datasheet
        POWER_MODE_REGISTER = 0x3E  # Power mode register address
        NORMAL_POWER_MODE = 0x00    # Normal power mode value
        #OP_MODE_REGISTER = 0x3D
        CONFIG_MODE = 0x00          # Configure mode value, goes into OP_Mode Register
        NDOF_MODE = 0x0C            # Nine Degrees of Freedom mode, 1100 in binary

        # Initialize the sensor - Set to CONFIG_MODE
        self.change_mode(CONFIG_MODE)

        # Set to normal power mode
        self.i2c.writeto_mem(self.addr, POWER_MODE_REGISTER, bytearray([NORMAL_POWER_MODE]))
        

        # Additional initial configurations (if required) go here

        # Switch back to a default operation mode - NDOF mode
        self.change_mode(NDOF_MODE)
        
    def change_mode(self, mode):
        OP_MODE_REGISTER = 0x3D
        self.i2c.writeto_mem(self.addr, OP_MODE_REGISTER, bytearray([mode]))     #Section 3.3 to see modes
        
    def reset_sensor(self):
        # Reset the sensor using the RST pin
        self.rst.value(0)  # Set pin low to reset
        pyb.delay(100)     # Wait for the reset to take effect (100 ms)
        self.rst.value(1)  # Bring the pin back high

    def read_euler_angles(self):
        # Default unit is Degrees
        EULER_H_LSB = 0x1A  # Register address for Euler Heading (x) LSB
        EULER_R_LSB = 0x1C  # Register address for Euler Roll (y) LSB
        EULER_P_LSB = 0x1E  # Register address for Euler Pitch (z) LSB

        # Read the Euler Heading (H), Roll (R), and Pitch (P) data
        # Note: Each component's data is 2 bytes (LSB and MSB)
        euler_h_data = self.i2c.readfrom_mem(self.addr, EULER_H_LSB, 2)
        euler_r_data = self.i2c.readfrom_mem(self.addr, EULER_R_LSB, 2)
        euler_p_data = self.i2c.readfrom_mem(self.addr, EULER_P_LSB, 2)

        # Convert each component's data
        euler_h = self.convert_data(euler_h_data)
        euler_r = self.convert_data(euler_r_data)
        euler_p = self.convert_data(euler_p_data)

        return (euler_h, euler_r, euler_p)


    def read_angular_velocity(self):
        # Default unit is Degrees/second
        GYRO_X_LSB = 0x14  # Replace with the actual register address for Gyroscope X-axis LSB
        GYRO_Y_LSB = 0x16  # Replace with the actual register address for Gyroscope Y-axis LSB
        GYRO_Z_LSB = 0x18  # Replace with the actual register address for Gyroscope Z-axis LSB

        # Read the Gyroscope X, Y, and Z data
        # Note: Each component's data is 2 bytes (LSB and MSB)
        gyro_x_data = self.i2c.readfrom_mem(self.addr, GYRO_X_LSB, 2)
        gyro_y_data = self.i2c.readfrom_mem(self.addr, GYRO_Y_LSB, 2)
        gyro_z_data = self.i2c.readfrom_mem(self.addr, GYRO_Z_LSB, 2)

        # Convert each component's data
        gyro_x = self.convert_data(gyro_x_data)
        gyro_y = self.convert_data(gyro_y_data)
        gyro_z = self.convert_data(gyro_z_data)

        return (gyro_x, gyro_y, gyro_z)


    def convert_data(self, data_bytes):
        # Convert two bytes into a single integer (consider endianness and signed format)
        value = (data_bytes[1] << 8) | data_bytes[0]
        if value & 0x8000:
            value -= 1 << 16
        return value
    
    def remap_axes(self):
        CONFIG_MODE = 0x00          # Configure mode value, goes into OP_Mode Register
        NDOF_MODE = 0x0C            # Nine Degrees of Freedom mode, 1100 in binary
        
        AXIS_MAP_CONFIG_REG = 0x41  # Axis map config register
        AXIS_MAP_SIGN_REG = 0x42    # Axis map sign register

        # Keep default axis mapping
        config_value = 0x24  # Default: X->X, Y->Y, Z->Z

        # Invert signs for X and Y axes
        sign_value = 0x06  # Binary 00000110: Invert X (Bit 2) and Y (Bit 1)  Makes Z up, X positive to the right, and Y positive to the front of the romi

        # Ensure we are in config mode before writing to registers
        self.change_mode(CONFIG_MODE)

        # Write to the axis remap registers
        self.i2c.writeto_mem(self.addr, AXIS_MAP_CONFIG_REG, bytearray([config_value]))
        self.i2c.writeto_mem(self.addr, AXIS_MAP_SIGN_REG, bytearray([sign_value]))

        # Switch back to a default operation mode - NDOF mode
        self.change_mode(NDOF_MODE)
        
    def get_calibration_status(self):
        CALIB_STAT_REGISTER = 0x35  # Calibration_Stat register address
        status = self.i2c.readfrom_mem(self.addr, CALIB_STAT_REGISTER, 1)
        status = status[0]           # First byte of the register
        sys = (status >> 6) & 0x03   # Shift and binary 11 to get 2 bits on right side
        gyro = (status >> 4) & 0x03
        accel = (status >> 2) & 0x03
        mag = status & 0x03
        return (sys, gyro, accel, mag)

    def get_calibration_coefficients(self):
        CALIB_COEFF_REGISTER = 0x55  # Replace with the actual register address Check
        coeffs = self.i2c.readfrom_mem(self.addr, CALIB_COEFF_REGISTER, 22)
        return coeffs
    
    def load_calibration(self):
        calib_file = 'IMU_cal_coeffs.txt'
        if calib_file in os.listdir():       #Checks if calib_file is in current directory
            with open(calib_file, 'r') as f:
                calib_data = f.read().split(',')
                calib_bytes = bytearray(int(x, 0) for x in calib_data)
                self.write_calibration_coefficients(calib_bytes)
        else:
            # Calibration file does not exist, initiate calibration process
            self.calibrate_imu()

    def calibrate_imu(self):
        print("Starting IMU calibration...")

        # Accelerometer Calibration
        print("Calibrate Accelerometer: Place in 6 positions, each for a few seconds.")
        #while not self.is_accelerometer_calibrated():
        pyb.delay(10000)  # Delay for visual/manual adjustment

        # Gyroscope Calibration
        print("Calibrate Gyroscope: Keep the sensor stationary for a few seconds.")
        #while not self.is_gyroscope_calibrated():
        pyb.delay(5000)

        # Magnetometer Calibration
        print("Calibrate Magnetometer: Perform random movements like '8' in the air.")
        #while not self.is_magnetometer_calibrated():
        pyb.delay(5000)

        print("IMU calibration complete.")

        # Read and save calibration data
        calib_data = self.get_calibration_coefficients()  # Implement this method
        with open('IMU_cal_coeffs.txt', 'w') as f:
            f.write(','.join('0x{:02x}'.format(x) for x in calib_data))

    def write_calibration_coefficients(self, coeffs):
        CALIB_COEFF_REGISTER = 0xBB  # Replace with the actual register address
        self.i2c.writeto_mem(self.addr, CALIB_COEFF_REGISTER, bytearray(coeffs))
        
    def check_calibration_status(self):
        CALIB_STAT_REGISTER = 0x35  # Calibration status register address

        status = self.i2c.readfrom_mem(self.addr, CALIB_STAT_REGISTER, 1)[0]
        sys_calib = (status >> 6) & 0x03
        gyro_calib = (status >> 4) & 0x03
        accel_calib = (status >> 2) & 0x03
        mag_calib = status & 0x03

        return {
            'System': sys_calib,
            'Gyroscope': gyro_calib,
            'Accelerometer': accel_calib,
            'Magnetometer': mag_calib
        }


'''

# Example usage
sda_pin = 'B8'  # SDA pin number
scl_pin = 'C9'  # SCL pin number
rst_pin = 'C2'  # RST pin number

# Create the BNO055 sensor instance with the specified pins
sensor = BNO055(sda_pin, scl_pin, rst_pin)

sensor.remap_axes()

# Load existing calibration or calibrate
sensor.load_calibration()

# Read and print the calibration status
calibration_status = sensor.check_calibration_status()
#print("Calibration Status:", calibration_status)

# Read and print the calibration coefficients
calibration_coefficients = sensor.get_calibration_coefficients()
print("Calibration Coefficients:", calibration_coefficients)

# Save the calibration coefficients to a file
with open('IMU_cal_coeffs.txt', 'w') as file:
    formatted_coeffs = ','.join('0x{:02x}'.format(coeff) for coeff in calibration_coefficients)
    file.write(formatted_coeffs)
    print("Calibration coefficients saved to 'IMU_cal_coeffs.txt'")



sensor.remap_axes()
sensor.load_calibration()
calibration_status = sensor.check_calibration_status()
print("Calibration Status:", calibration_status)

while True:
    euler_angles = sensor.read_euler_angles()
    angular_velocity = sensor.read_angular_velocity()

    print("Euler Angles:", euler_angles)
    print("Angular Velocity:", angular_velocity)

    pyb.delay(1000)  # Delay for 500 milliseconds (0.5 seconds)'''
