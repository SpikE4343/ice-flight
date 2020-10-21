import csv
import sys
import os
import serial

import numpy as np
import binascii
import struct
import time
import bitstruct

import scipy.fftpack
import fport_tx
import random

from lognplot.client import LognplotTcpClient

#baud = 115200 # 2000000
baud = 2000000

client = LognplotTcpClient()
client.connect()

use_file = len(sys.argv) > 1

if use_file:
    inputFile = open(sys.argv[1], "rb")
else:
    ser = serial.Serial(port='/dev/ttyUSB0', baudrate=baud, stopbits=1)
    ser.flushInput()

size = 1024
pause_time = 0.1

msgGyro = struct.Struct('< i i i')

msgRx = struct.Struct('< H H H H H B B')
msgMotor = struct.Struct('< i i i i')
msgInput = struct.Struct('< i i i i i')
msgSetpoint = struct.Struct('< i i i i i')

msgStatus = bitstruct.compile('u8u8u8u8u8u8u8')

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



def last(l):
    s = len(l)
    if (s > 0):
        return l[s - 1]
    return None


def readData():
    global readByteData
    if use_file:
        for d in inputFile.read():
            readByteData.append(d)
            print(d)

    elif (ser.in_waiting > 0):
        for d in ser.read(ser.in_waiting):
            readByteData.append(d)


lt = time.time()
def writeData():
  global lt
  if(time.time() - lt < 0.01):
    return

  lt = time.time()
  d = fport_tx.writeControls([random.random()*1792]*16, 0, 0)
  #print(time.time() - lt)
  ser.write(bytearray(d))


gyroMsgCount = 0

def updateData():
    global readByteData
    global gyroMsgCount

    readData()

    if(len(readByteData) <= 0):
      return

    readBufferLenData.append(len(readByteData))
    if (len(readBufferLenData) > size):
        readBufferLenData.pop(0)

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

        t = time.time()
        #print(msgType, msgLen, len(ser_bytes), len(readByteData) + 3)

        readByteData = readByteData[(3 + msgLen):]

        if msgType == 6 and msgInput.size <= len(ser_bytes):
            setpoint = msgSetpoint.unpack(ser_bytes)
            for i in range(5):
                 client.send_sample(f"setpoint_{i}", t, setpoint[i])

        elif msgType == 5 and 7 == len(ser_bytes):
          
          status = msgStatus.unpack(ser_bytes)
          print(status)
          client.send_sample(f"state", t, int(status[0]))
          client.send_sample(f"armed", t, int(status[1]))
          client.send_sample(f"failsafe", t, int(status[2]))
          client.send_sample(f"rxFrameLoss", t, int(status[3]))
          client.send_sample(f"decode_state", t, int(status[4]))
          client.send_sample(f"motor_hz", t, int(status[5]))
          client.send_sample(f"gyro_hz", t, int(status[6]))

        # Gyro
        elif msgType == 4 and msgGyro.size == len(ser_bytes):
            gyroMsgCount += 1
            gyro = msgGyro.unpack(ser_bytes)
            for i in range(3):
                client.send_sample(f"gyro_{i}", gyroMsgCount, gyro[i]/ (2**28) * 2000)

        # RX
        elif msgType == 3 and msgRx.size == len(ser_bytes):
            rx = msgRx.unpack(ser_bytes)
            rxData.append(rx)
            for i in range(5):
               client.send_sample(f"rx_{i}", t, rx[i])

            client.send_sample(f"rx_rssi", t, rx[5])
            client.send_sample(f"rx_failsafe", t, rx[6])
            

        # Motors
        elif msgType == 2 and msgMotor.size == len(ser_bytes):
            motor = msgMotor.unpack(ser_bytes)
            for i in range(4):
                client.send_sample(f"motor_{i}", t, motor[i]/ (2**28) * 2000)

        #Inputs

        elif msgType == 1 and msgInput.size == len(ser_bytes):
            inputs = msgInput.unpack(ser_bytes)
            for i in range(4):
                client.send_sample(f"input_{i}", t, inputs[i]/ (2**28) * 2000)



#TODO: async block until serial port has data
while(True):
  time.sleep(0.01)
  updateData()
  writeData()