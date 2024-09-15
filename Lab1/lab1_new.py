import serial
import io
import pyqtgraph as pg
import numpy as np
import array
import math
from collections import deque
import threading

WINDOW_SIZE = 19
ACC_THRESHOLD = 0.001
GYRO_THRESHOLD = 0.001

acc_div = 16384.0
accx_offset = 0.230
accy_offset = 0.0
accz_offset = -2.0

gyro_div = 131.0
gyrox_offset = 0.230
gyroy_offset = 0.0
# gyroz_offset = 0.0

###### ONLY FOR PART A ######
# Low-pass filter coefficient (adjust for desired filtering strength)
alpha = 0.5  # Smoothing factor between 0 (max smoothing) and 1 (no smoothing)
prev_acc = [0, 0, 0]
#############################

########## ONLY FOR PART B ##########
# Initialize complementary filter parameters
alpha_filter = 0.9  # Complementary filter coefficient
dt = 0.01  # Time step (adjust based on your sensor data rate)
previous_roll = 0
previous_pitch = 0
#####################################

velocityx = 0.0
velocityy = 0.0
displacementx = 0.0
displacementy = 0.0

data_lock = threading.Lock()

Data = serial.Serial('COM9', 57600, timeout=0.01)
# Data.open()
app = pg.mkQApp()
win = pg.GraphicsLayoutWidget()
win.setWindowTitle('Lab 1')
win.resize(1600, 900)
xLength = 300

fig1 = win.addPlot()
fig1.showGrid(x=True, y=True)
fig1.setRange(xRange = [0, xLength], yRange=[-30000 / acc_div, 30000 / acc_div], padding=0)
fig1.setLabel(axis='left', text='g')
fig1.setLabel(axis='bottom', text='x / point')
fig1.setTitle('Acc Data')

curve1 = fig1.plot()
curve2 = fig1.plot()
curve3 = fig1.plot()

# Deques for storing data
data = [deque(maxlen=xLength) for _ in range(5)]
stationary_data = [[0] * WINDOW_SIZE for _ in range(5)]

running_averages = [0] * 5  # Store the running averages for each deque

###### ONLY FOR PART A ######
def low_pass_filter(new_value, prev_filtered_value, alpha):
    """Applies low-pass filter to a signal."""
    return alpha * new_value + (1 - alpha) * prev_filtered_value

def detect_stationary_accel(data, threshold=ACC_THRESHOLD):
    # Ensure that each accelerometer deque has at least 50 elements
    if any(len(data[i]) < WINDOW_SIZE for i in range(3)):  # accX, accY, accZ
        return False  # Not enough data to check
    
    # Slice the latest WINDOW_SIZE values from accX, accY, and accZ deques
    accX_window = list(data[0])[-WINDOW_SIZE:]
    accY_window = list(data[1])[-WINDOW_SIZE:]
    accZ_window = list(data[2])[-WINDOW_SIZE:]
    
    # Stack the latest values for the three axes into a NumPy array
    accel_window = np.column_stack((accX_window, accY_window, accZ_window))
    
    # Calculate the standard deviation for each axis
    accel_var = np.var(accel_window, axis=0)
    print(accel_var)
    
    # Return True if the std deviation is below the threshold for all axes
    return np.all(accel_var < threshold)
##############################

########## ONLY FOR PART B ##########
def detect_stationary_accel_gyro(data, recent_data, accel_threshold=ACC_THRESHOLD, gyro_threshold=GYRO_THRESHOLD):
    # Ensure that each accelerometer and gyroscope deque has at least 50 elements
    if any(len(data[i]) < WINDOW_SIZE for i in range(5)):  # accX, accY, accZ, gyroX, gyroY
        return False  # Not enough data to check
    
    # Slice the latest WINDOW_SIZE values from each deque
    accX_window = list(data[0])[-WINDOW_SIZE:] + [recent_data[0]]
    accY_window = list(data[1])[-WINDOW_SIZE:] + [recent_data[1]]
    accZ_window = list(data[2])[-WINDOW_SIZE:] + [recent_data[2]]
    gyroX_window = list(data[3])[-WINDOW_SIZE:] + [recent_data[3]]
    gyroY_window = list(data[4])[-WINDOW_SIZE:] + [recent_data[4]]
    
    # Stack the latest values for the three accelerometer axes into a NumPy array
    accel_window = np.column_stack((accX_window, accY_window, accZ_window))
    # Stack the latest values for the two gyroscope axes into a NumPy array
    gyro_window = np.column_stack((gyroX_window, gyroY_window))
    
    # Calculate the standard deviation for each axis
    accel_var = np.var(accel_window, axis=0)
    gyro_var = np.var(gyro_window, axis=0)
    print(f"Accel_var: {accel_var}, Gyro_var: {gyro_var}, result: {np.all(accel_var > accel_threshold) and np.all(gyro_var > gyro_threshold)}")
    
    # Return True if both accelerometer and gyroscope variance are below their respective thresholds
    return np.all(accel_var > accel_threshold) and np.all(gyro_var > gyro_threshold)
#####################################

def update_running_average(index, new_value):
    """Efficiently updates the running average when new data is added."""
    global running_averages

    if len(data[index]) == data[index].maxlen:
        # If the deque is full, remove the oldest value from the average
        oldest_value = data[index][0]
        running_averages[index] -= oldest_value / data[index].maxlen

    # Add the new value to the average
    running_averages[index] += new_value / data[index].maxlen

def get_similar_values():
    """Returns the running averages to handle corrupted data."""
    return running_averages

def canFloat(data):
    try:
        float(data)
        return True
    except ValueError:
        return False

def dataProcess(signal):
    global data
    global prev_acc # Access previous filtered values PART A
    global previous_roll, previous_pitch # Access previous roll and pitch values PART B
    signal = signal.strip()  # Remove any leading/trailing whitespace
    dataSet = []
    datapoint = ''
    for i in signal:
        if i.isdigit() or i == '-': 
            datapoint += i
        elif i == ',':
            if canFloat(datapoint):
                dataSet.append(float(datapoint))
            datapoint = ''
    
    # Handle the last datapoint if no trailing comma
    if datapoint and canFloat(datapoint):
        dataSet.append(float(datapoint))

    
    if len(dataSet) == 5:
        #Calibration with just acceleration data
        accel_x = dataSet[0] / acc_div - accx_offset
        accel_y = dataSet[1] / acc_div - accy_offset
        accel_z = dataSet[2] / acc_div - accz_offset

        # Calibration for gyroscope data
        gyro_x = dataSet[3] / gyro_div - gyrox_offset
        gyro_y = dataSet[4] / gyro_div - gyroy_offset
        # gyro_z = dataSet[5] / gyro_div - gyroz_offset

        ############## FOR PART A ################
        # # Apply low-pass filter to accelerometer data
        # accel_x = low_pass_filter(accel_x, prev_acc[0], alpha)
        # accel_y = low_pass_filter(accel_y, prev_acc[1], alpha)
        # accel_z = low_pass_filter(accel_z, prev_acc[2], alpha)
        # prev_acc = [accel_x, accel_y, accel_z]
        #########################################

        # Compute accelerometer-based pitch and roll
        pitch = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2))    
        roll = math.atan2(accel_x, math.sqrt(accel_y**2 + accel_z**2))

        ############## FOR PART B ################
        gyro_x = math.radians(gyro_x)
        gyro_y = math.radians(gyro_y)
        # gyro_z = math.radians(gyro_z)
        
        # Integrate gyroscope data (gyro_x and gyro_y are rates of rotation in rad/s)
        gyro_pitch = previous_pitch + gyro_y * dt
        gyro_roll = previous_roll + gyro_x * dt

        # Apply complementary filter
        pitch = alpha * gyro_pitch + (1 - alpha) * pitch
        roll = alpha * gyro_roll + (1 - alpha) * roll

        # Update previous roll and pitch
        previous_pitch, previous_roll = pitch, roll
        #########################################

        offset_accx = math.sin(roll)
        offset_accy = -math.sin(pitch)
        offset_accz = math.cos(pitch) * math.cos(roll)

        # Subtract gravity component
        a_x = accel_x - offset_accx
        a_y = accel_y - offset_accy
        a_z = accel_z - offset_accz

        new_data = [a_x, a_y, a_z, gyro_x, gyro_y]
        print(f"New data: {new_data}")
        ############## CHEAT CODE ################
        if not detect_stationary_accel_gyro(data, new_data):
            return [0, 0, 0, 0, 0]

        return new_data
    
    # else: # Return Averages if data is corrupted
    #     return get_similar_values()

##################### Thread to read data from serial port ####################
def readData():
    global signal
    global velocityx, velocityy, displacementx, displacementy
    while True:
        try:
            signal = Data.readline().decode('utf-8').strip()
            signal = dataProcess(signal)
            if signal:
                # print(f"Signal: {signal}")

                # Integrate acceleration to get velocity
                # velocityx += signal[0] * dt
                # velocityy += signal[1] * dt
                # # Integrate velocity to get displacement
                # displacementx += velocityx * dt
                # displacementy += velocityy * dt
                # print(f"X: {displacementx}, Y: {displacementy}")
                with data_lock:
                    for i in range(len(data)):
                        data[i].append(signal[i])
                        # update_running_average(i, signal[i])

        except serial.SerialException as e:
                print(f"Serial exception: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

# Start the data reading thread
data_thread = threading.Thread(target=readData, daemon=True)
data_thread.start()

# Now, only handle plotting in the main thread
def plotData():
    curve1.setData(list(data[0]), pen=pg.mkPen('g', width=2))
    curve2.setData(list(data[1]), pen=pg.mkPen('r', width=2))
    curve3.setData(list(data[2]), pen=pg.mkPen('b', width=2))

##############################################################################


timer = pg.QtCore.QTimer()
timer.timeout.connect(plotData)
win.show()
timer.start(1) # Increase the value to fix the lagging issue
app.exec()