import csv
import sys
import os
import serial

ser = serial.Serial(port='/dev/ttyUSB0', baudrate=256000, stopbits=1)
ser.flushInput()

while True:
    try:
        print("waiting for data")
        ser_bytes = ser.read(3)

        decoded_bytes = ser_bytes.decode("utf-8")
        print(ser_bytes[0], ser_bytes[1], ser_bytes[2])
    except:
        print("Keyboard Interrupt")
        break