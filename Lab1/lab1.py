import serial
import io
import pyqtgraph as pg
import numpy as np
import array
import math

Data = serial.Serial('COM13', 57600, timeout=0.01)
# Data.open()
app = pg.mkQApp()
win = pg.GraphicsLayoutWidget()
win.setWindowTitle('Lab 1')
win.resize(1600, 900)
xLength = 300

# sensitivity = 16384
# sensitivity = 2048
sensitivity = 1

fig1 = win.addPlot()
fig1.showGrid(x=True, y=True)
fig1.setRange(xRange=[0, xLength], yRange=[-30000, 30000], padding=0) #/ 16384
fig1.setLabel(axis='left', text='g')
fig1.setLabel(axis='bottom', text='x / point')
fig1.setTitle('acceleration')

curve1 = fig1.plot()
curve2 = fig1.plot()
curve3 = fig1.plot()

ax = array.array('d')
ay = array.array('d')
az = array.array('d')

data = [np.zeros(xLength).__array__('d'), np.zeros(xLength).__array__('d'), np.zeros(xLength).__array__('d')]

def canFloat(data):
    try:
        float(data)
        return True
    except ValueError:
        return False

def dataProcess(data):
    data = str(data)
    dataSet = []
    datapoint = ''
    for i in data:
        if i.isdigit() or i == '-':
            datapoint += i
        elif i == ',' and canFloat(datapoint):
            dataSet.append(float(datapoint) / sensitivity)
            datapoint = ''
    return dataSet

def plotData():
    global signal
    signal = Data.readline()
    print("Original Signal", signal)
    signal = dataProcess(signal)
    print("Processed Signal", signal)
    if len(signal) == 3:
        print("Appending to Graph")
        for i in range(len(data)):
            if len(data[i]) < xLength:
                data[i].append(signal[i])
            else:
                data[i][:-1] = data[i][1:]
                data[i][-1] = signal[i]

        ax.append(signal[0])
        ay.append(signal[1])
        az.append(signal[2])

        curve1.setData(data[0], pen=pg.mkPen('g', width=3))
        curve2.setData(data[1], pen=pg.mkPen('r', width=3))
        curve3.setData(data[2], pen=pg.mkPen('b', width=3))
    else:
        print("Unusable")
    print("----------------------------------------------------------------------------")

timer = pg.QtCore.QTimer()
timer.timeout.connect(plotData)
win.show()
timer.start(1)
app.exec()

# try:
#     input("Press Enter to stop\n")
# except (Exception, KeyboardInterrupt):
#     pass

# np.savetxt('x_acc.txt', ax)
# np.savetxt('y_acc.txt', ay)
# np.savetxt('z_acc.txt', az)

# while True:
#     data = Data.readline()
#     print(dataProcess(data))