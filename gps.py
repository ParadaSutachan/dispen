import serial
import time
import string
import pynmea2

while True:
    port="/dev/ttyAMA0"
    ser=serial.Serial(port, baudrate=9600, timeout=0.1)
    dataout = pynmea2.NMEAStreamReader()
    newdata=ser.readline().decode('utf-8').strip()
    newmsg=pynmea2.parse(newdata)
    if newmsg.status == 'V':
        print('Estoy Agarrando señal Krnarl...')

    while newmsg.status == 'A':
        if newdata[0:6] == "$GPRMC":
            print("Ya tengo señal Krnal ....")
            print("Toma tus datos .... ")
            newmsg=pynmea2.parse(newdata)
            lat=newmsg.latitude
            lng=newmsg.longitude
            gps = "Lat = " + str(lat) + " Lng = " + str(lng)
            print(gps)
            speed = newmsg.spd_over_grnd
            #speed_mps= speed_kmph*(1000/3600)
            print(f"Speed : {speed:.2f} m/s " )
            #speed_mps = speed_kmph * (1000 / 3600)
            #print(f"Speed (horizontal): {speed_mps:.2f} m/s")
		