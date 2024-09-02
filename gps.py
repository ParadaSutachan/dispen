import serial
import time
import string
import pynmea2

while True:
        port="/dev/ttyAMA0"
        ser=serial.Serial(port, baudrate=9600, timeout=0.5)
        dataout = pynmea2.NMEAStreamReader()
        newdata=ser.readline()
        if newdata[0:6] == '$GPRMC':
            print(1)