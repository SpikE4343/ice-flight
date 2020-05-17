import csv
import sys
import os
import serial

import numpy as np
import binascii
import struct

from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy as np

ser = serial.Serial(port='/dev/ttyUSB0', baudrate=400000, stopbits=1)
ser.flushInput()

size = 1000
pause_time = 0.1

# msgGyro = struct.Struct('< h h h')
msgGyro = struct.Struct('< h h h')
msgRx = struct.Struct('< H H H H H B B')
msgMotor = struct.Struct('< h h h h')
msgInput = struct.Struct('< h h h h h')

x = np.arange(size)

gyroData = [list(range(size)), list(range(size)), list(range(size))]
rxData = []
motorData = []
inputData = []

# plt.ion()
# gyroFig = plt.figure(figsize=(13, 6))
# gyroAx = gyroFig.add_subplot(111)
# create a variable for the line so we can later update it

# x_vec = np.linspace(0, 1, size + 1)[0:-1]
# gyroline1, = gyroAx.plot(x_vec, gyroData, '-o', alpha=0.8)
#update plot label/title
# plt.ylabel('Y Label')
# plt.title('Title: Gyro')
# plt.show()

index = 0

gyroPlot = pg.plot(title="Gyro [R,P,Y] Deg/s")
gyroPlot.setYRange(-3000, 3000, padding=None, update=True)
gyroCurves = []
for i in range(3):
    c = gyroPlot.plot(x, gyroData[i], pen=(i, 3))
    print(c)
    gyroCurves.append(c)

# for i in range(3):
#     plotWidget.plot(x, y[i], pen=(i,3))  ## setting pen=(i,3) automaticaly creates three different-colored pens


def last(l):
    s = len(l)
    if (s > 0):
        return l[s - 1]
    return None


while True:
    try:
        QtGui.QApplication.processEvents()
        #print("waiting for data")
        marker = ser.read(1)[0]
        # print("marker:", marker)
        if (marker != 127):
            continue

        msgType = ser.read(1)[0]
        msgLen = ser.read(1)[0]

        ser_bytes = ser.read(msgLen)

        #decoded_bytes = ser_bytes.decode("utf-8")
        #print(ser_bytes[0], ser_bytes[1], ser_bytes[2])
        # print("MSG:", msgType, "LEN:", msgLen)
        if msgType == 4:
            gyro = msgGyro.unpack(ser_bytes)
            gyro = (int(gyro[0] / 16.4 * 2), int(gyro[1] / 16.4 * 2),
                    int(gyro[2] / 16.4 * 2))

            for i in range(3):
                gyroData[i].append(gyro[i])
                if (len(gyroData[i]) > size):
                    gyroData[i].pop(0)

                gyroCurves[i].setData(gyroData[i])

            # print(ser_bytes)
            # gyroline1.set_ydata(gyroData)
            # plt.pause(pause_time)
            # print("gyro:  ", gyro)
        elif msgType == 3:
            rx = msgRx.unpack(ser_bytes)
            rxData.append(rx)
            # print(ser_bytes)
            if (len(motorData) > size):
                rxData.pop(0)
            # print("rx:    ", rx)
        elif msgType == 2:
            motor = msgMotor.unpack(ser_bytes)
            motorData.append(motor)
            if (len(motorData) > size):
                motorData.pop(0)
            # print("motor: ", motor)
        elif msgType == 1:
            inputs = msgInput.unpack(ser_bytes)
            # print(ser_bytes)
            inputData.append(inputs)
            if (len(inputData) > size):
                inputData.pop(0)

            g = last(gyroData)
            rx = last(rxData)
            m = last(motorData)
            inputs = last(inputData)
            # print("| g:", g, "|\trx:", rx, "|\ti:", inputs, "|\tm:", m,
            #       ser.in_waiting)

    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        break
    except Exception as e:
        print("error:", e)
        break

# import datetime as dt
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# import tmp102

# # Create figure for plotting
# fig = plt.figure()
# ax = fig.add_subplot(1, 1, 1)
# xs = []
# ys = []

# # Initialize communication with TMP102
# tmp102.init()

# # This function is called periodically from FuncAnimation
# def animate(i, xs, ys):

#     # Read temperature (Celsius) from TMP102
#     temp_c = round(tmp102.read_temp(), 2)

#     # Add x and y to lists
#     xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
#     ys.append(temp_c)

#     # Limit x and y lists to 20 items
#     xs = xs[-20:]
#     ys = ys[-20:]

#     # Draw x and y lists
#     ax.clear()
#     ax.plot(xs, ys)

#     # Format plot
#     plt.xticks(rotation=45, ha='right')
#     plt.subplots_adjust(bottom=0.30)
#     plt.title('TMP102 Temperature over Time')
#     plt.ylabel('Temperature (deg C)')

# # Set up plot to call animate() function periodically
# ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
# plt.show()