############## Accelerometer Data Collection ##############
import numpy as np

# Assume a function get_acceleration() returns the raw accelerometer values
# Calibration values (gravity components on each axis)
gravity_x = 0.0  # Calibrate with stationary IMU
gravity_y = 0.0
gravity_z = 9.8  # Gravitational acceleration along the z-axis (or based on the calibration)

# Low-pass filter implementation
def low_pass_filter(current_value, previous_value, alpha=0.8):
    return alpha * previous_value + (1 - alpha) * current_value

# Real-time acceleration readings
def process_acceleration():
    raw_acc = get_acceleration()  # Get raw accelerometer readings (x, y, z)
    acc_x, acc_y, acc_z = raw_acc
    
    # Remove gravity
    filtered_x = low_pass_filter(acc_x - gravity_x, acc_x)
    filtered_y = low_pass_filter(acc_y - gravity_y, acc_y)
    filtered_z = low_pass_filter(acc_z - gravity_z, acc_z)

    return filtered_x, filtered_y, filtered_z
############################################################

############# Accelerometer and Gyroscope Data Collection #############
import math

# Assume get_gyroscope() and get_acceleration() functions provide raw data

# Gyroscope integration for orientation (roll, pitch)
def calculate_orientation(gyro_data, dt):
    roll_rate, pitch_rate, yaw_rate = gyro_data
    # Integrate angular velocities over time to get angles (roll, pitch, yaw)
    roll += roll_rate * dt
    pitch += pitch_rate * dt
    return roll, pitch

# Gravity compensation based on IMU orientation
def compensate_gravity(accel_data, roll, pitch):
    acc_x, acc_y, acc_z = accel_data
    
    # Calculate the gravity component on each axis based on roll and pitch angles
    g_x = -9.8 * math.sin(pitch)
    g_y = 9.8 * math.sin(roll) * math.cos(pitch)
    g_z = 9.8 * math.cos(roll) * math.cos(pitch)
    
    # Subtract gravity from accelerometer data
    acc_x_no_gravity = acc_x - g_x
    acc_y_no_gravity = acc_y - g_y
    acc_z_no_gravity = acc_z - g_z
    
    return acc_x_no_gravity, acc_y_no_gravity, acc_z_no_gravity

# Combine accelerometer and gyroscope
def process_imu_data():
    accel_data = get_acceleration()
    gyro_data = get_gyroscope()
    
    dt = 0.01  # Assume a time step
    
    # Calculate orientation from gyroscope data
    roll, pitch = calculate_orientation(gyro_data, dt)
    
    # Compensate for gravity using orientation
    filtered_acc = compensate_gravity(accel_data, roll, pitch)
    
    return filtered_acc
######################################################################