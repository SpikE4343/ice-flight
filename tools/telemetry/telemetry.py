import csv
import sys
import os
import serial

import numpy as np
import binascii
import struct

from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import scipy.fftpack

ser = serial.Serial(port='/dev/ttyUSB0', baudrate=2000000, stopbits=1)
ser.flushInput()

size = 4096
pause_time = 0.1

msgGyro = struct.Struct('< i i i')

msgRx = struct.Struct('< H H H H H B B')
msgMotor = struct.Struct('< i i i i')
msgInput = struct.Struct('< i i i i i')
msgSetpoint = struct.Struct('< i i i i i')

x = np.arange(size)

gyroData = [[0] * size, [0] * size, [0] * size]
gyroFFtData = [[0] * size, [0] * size, [0] * size]

inputsData = [[0] * size, [0] * size, [0] * size, [0] * size]
setpointData = [[0] * size, [0] * size, [0] * size, [0] * size]
motorData = [[0] * size, [0] * size, [0] * size, [0] * size]
rxData = [[0] * size, [0] * size, [0] * size, [0] * size]

inputData = []
readByteData = bytearray()

serialBufferData = [0] * size
readBufferLenData = [0] * size

app = QtGui.QApplication([])
mw = QtGui.QMainWindow()
mw.setWindowTitle('Telemetry')
mw.resize(800, 800)
cw = QtGui.QWidget()
mw.setCentralWidget(cw)
l = QtGui.QVBoxLayout()
cw.setLayout(l)

# GYRO
gyroPlot = pg.PlotWidget(name='Gyro')
gyroPlot.setLabel('left', 'Gyro', units='deg/s')

gyroCurves = []
for i in range(3):
    c = pg.PlotDataItem(x, gyroData[i], pen=(i, 3))
    gyroPlot.addItem(c)
    gyroCurves.append(c)

# GYRO FFT
gyroFftPlot = pg.PlotWidget(name='Gyro FFT')
gyroFftPlot.setLabel('left', 'Gyro Freq', units='hz')

gyroFftCurves = []
for i in range(3):
    c = pg.PlotDataItem(x, gyroFFtData[i], pen=(i, 3))
    gyroFftPlot.addItem(c)
    gyroFftCurves.append(c)

# Motors
motorPlot = pg.PlotWidget(name='Motor')
motorPlot.setLabel('left', 'Motor', units='t')

motorCurves = []
for i in range(4):
    c = pg.PlotDataItem(x, motorData[i], pen=(i, 4))
    motorPlot.addItem(c)
    motorCurves.append(c)

# RX data
inputsPlot = pg.PlotWidget(name='Inputs')
inputsPlot.setLabel('left', 'Inputs', units='deg/s')

inputsCurves = []
for i in range(4):
    c = inputsPlot.plot(x, inputsData[i], pen=(i, 4))
    inputsCurves.append(c)

# RX data
setpointPlot = pg.PlotWidget(name='Set Point')
setpointPlot.setLabel('left', 'Set Point', units='deg/s')

setpointCurves = []
for i in range(4):
    c = setpointPlot.plot(x, setpointData[i], pen=(i, 4))
    setpointCurves.append(c)

# RX data
rxPlot = pg.PlotWidget(name='RX [T,A,E,R]')
rxPlot.setLabel('left', 'Rx', units='deg/s')

rxCurves = []
for i in range(4):
    c = rxPlot.plot(x, rxData[i], pen=(i, 4))
    rxCurves.append(c)

l.addWidget(gyroPlot)
l.addWidget(gyroFftPlot)
l.addWidget(rxPlot)
l.addWidget(setpointPlot)
l.addWidget(inputsPlot)
l.addWidget(motorPlot)

#Serial UART
# serialPlot = pg.PlotWidget(name='UART')
# serialPlot.setLabel('left', 'UART', units='byte')
# l.addWidget(serialPlot)
# c = pg.PlotDataItem(x, serialBufferData, pen=(0, 2))
# serialPlot.addItem(c)
# serialCurve = c

# c = pg.PlotDataItem(x, readBufferLenData, pen=(1, 2))
# serialPlot.addItem(c)
# rbCurve = c

mw.show()


def last(l):
    s = len(l)
    if (s > 0):
        return l[s - 1]
    return None


def updateData():
    global readByteData

    # serialBufferData.append(ser.in_waiting)
    # if len(serialBufferData) > size:
    #     serialBufferData.pop(0)

    # serialCurve.setData(serialBufferData)

    if (ser.in_waiting > 0):
        for d in ser.read(ser.in_waiting):
            readByteData.append(d)

    readBufferLenData.append(len(readByteData))
    if (len(readBufferLenData) > size):
        readBufferLenData.pop(0)

    # rbCurve.setData(readBufferLenData)

    count = 0
    while (count < size and len(readByteData) > 0):
        marker = readByteData[0]
        count += 1
        #print(marker)
        if (marker != 127):
            readByteData.pop(0)
            continue

        if (len(readByteData) < 4):
            continue

        msgType = readByteData[1]
        msgLen = readByteData[2]

        if (msgLen <= 0):
            readByteData.pop(0)
            continue

        if (msgLen > len(readByteData) + 3):
            continue

        ser_bytes = readByteData[3:3 + msgLen]

        #print(msgType, msgLen, len(ser_bytes), len(readByteData) + 3)

        readByteData = readByteData[(3 + msgLen):]

        if msgType == 6 and msgInput.size <= len(ser_bytes):
            setpoint = msgSetpoint.unpack(ser_bytes)
            for i in range(4):
                setpointData[i].append(setpoint[i])
                if (len(setpointData[i]) > size):
                    setpointData[i].pop(0)
                # inputsCurves[i].setData(inputsData[i])
        # Gyro
        elif msgType == 4 and msgGyro.size == len(ser_bytes):

            gyro = msgGyro.unpack(ser_bytes)
            # gyro = (int(gyro[0] / 16.4 * 2), int(gyro[1] / 16.4 * 2),
            #         int(gyro[2] / 16.4 * 2))
            for i in range(3):
                #print(i, gyro[i], gyro[i] / (2**28))
                gyroData[i].append(gyro[i] / (2**28))
                gyroFFtData[i].append(gyro[i] / (2**28))
                if (len(gyroData[i]) > size):
                    gyroData[i].pop(0)
                    gyroFFtData[i].pop(0)
                yf = scipy.fftpack.fft(gyroFFtData[i])
                gyroFftCurves[i].setData(2.0 / size * np.abs(yf[:size // 2]))

        # RX
        elif msgType == 3 and msgRx.size == len(ser_bytes):
            rx = msgRx.unpack(ser_bytes)
            rxData.append(rx)
            for i in range(4):
                rxData[i].append(rx[i])
                if (len(rxData[i]) > size):
                    rxData[i].pop(0)

        # Motors
        elif msgType == 2 and msgMotor.size == len(ser_bytes):
            motor = msgMotor.unpack(ser_bytes)
            for i in range(4):
                motorData[i].append(motor[i])
                if (len(motorData[i]) > size):
                    motorData[i].pop(0)

        #Inputs

        elif msgType == 1 and msgInput.size == len(ser_bytes):
            inputs = msgInput.unpack(ser_bytes)
            for i in range(4):
                inputsData[i].append(inputs[i])
                if (len(inputsData[i]) > size):
                    inputsData[i].pop(0)
                # inputsCurves[i].setData(inputsData[i])

    for i in range(3):
        gyroCurves[i].setData(gyroData[i])

    for i in range(4):
        setpointCurves[i].setData(setpointData[i])
        inputsCurves[i].setData(inputsData[i])
        rxCurves[i].setData(rxData[i])
        motorCurves[i].setData(motorData[i])


t = QtCore.QTimer()
t.timeout.connect(updateData)
t.start(1)

app.exec()