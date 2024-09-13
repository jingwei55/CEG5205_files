import serial
import io
import pyqtgraph as pg
import numpy as np
import array
import math
from collections import deque
import threading

acc_div = 16384
gyro_div = 131
accx_offset = 0.190
accy_offset = -0.009
accz_offset = -1.020

gyrox_offset = 0.0
gyroy_offset = 0.0
gyroz_offset = 0.0

data_lock = threading.Lock()

Data = serial.Serial('COM13', 57600, timeout=0.01)
# Data.open()
app = pg.mkQApp()
win = pg.GraphicsLayoutWidget()
win.setWindowTitle('Lab 1')
win.resize(1600, 900)
xLength = 300

fig1 = win.addPlot()
fig1.showGrid(x=True, y=True)
fig1.setRange(xRange = [0, xLength], yRange=[-30000 / 16384, 30000 / 16384], padding=0)
fig1.setLabel(axis='left', text='g')
fig1.setLabel(axis='bottom', text='x / point')
fig1.setTitle('Accelerometer Data')

curve1 = fig1.plot()
curve2 = fig1.plot()
curve3 = fig1.plot()

ax = array.array('d')
ay = array.array('d')
az = array.array('d')

# data = [np.zeros(xLength).__array__('d'), np.zeros(xLength).__array__('d'), np.zeros(xLength).__array__('d')]
data = [deque(maxlen=xLength), deque(maxlen=xLength), deque(maxlen=xLength)]
running_averages = [0, 0, 0]  # Store the running averages for each deque

def update_running_average(index, new_value):
    """Efficiently updates the running average when new data is added."""
    global running_averages

    if len(data[index]) == data[index].maxlen:
        # If the deque is full, remove the oldest value from the average
        oldest_value = data[index][0]
        running_averages[index] -= oldest_value / data[index].maxlen

    # Add the new value to the average
    running_averages[index] += new_value / data[index].maxlen

def canFloat(data):
    try:
        float(data)
        return True
    except ValueError:
        return False
    
def get_similar_values():
    """Returns the running averages to handle corrupted data."""
    return running_averages

def dataProcess(signal):
    signal = signal.strip()  # Remove any leading/trailing whitespace
    dataSet = []
    datapoint = ''
    for i in signal:
        if i.isdigit() or i == '-' or i == '.': 
            datapoint += i
        elif i == ',':
            if canFloat(datapoint):
                dataSet.append(float(datapoint) / 16384)
            datapoint = ''
    
    # Handle the last datapoint if no trailing comma
    if datapoint and canFloat(datapoint):
        dataSet.append(float(datapoint) / 16384)

    
    if len(dataSet) == 3:
        dataSet[0] -= accx_offset
        dataSet[1] -= accy_offset
        dataSet[2] -= accz_offset
        return dataSet
    else:
        return get_similar_values()

##################### Thread to read data from serial port ####################
def readData(): #plotData
    global signal
    while True:
        try:
            signal = Data.readline().decode('utf-8').strip()
            signal = dataProcess(signal)
            # if signal[0] < 500:
            print(signal)
            # if len(signal) == 3:
            if signal:
                with data_lock:
                    for i in range(len(data)):
                        data[i].append(signal[i])
                        update_running_average(i, signal[i])

        except serial.SerialException as e:
                print(f"Serial exception: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

        # ax.append(signal[0])
        # ay.append(signal[1])
        # az.append(signal[2])

# Start the data reading thread
data_thread = threading.Thread(target=readData, daemon=True)
data_thread.start()

# Now, only handle plotting in the main thread
def plotData():
        curve1.setData(list(data[0]), pen=pg.mkPen('g', width=3))
        curve2.setData(list(data[1]), pen=pg.mkPen('r', width=3))
        curve3.setData(list(data[2]), pen=pg.mkPen('b', width=3))
##############################################################################

timer = pg.QtCore.QTimer()
timer.timeout.connect(plotData)
win.show()
timer.start(20) # Increase the value to fix the lagging issue
app.exec()