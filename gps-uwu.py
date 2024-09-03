# -- coding: utf-8 --
#!/usr/bin/env python3

import serial  #type: ignore
import time  
import pynmea2 #type: ignore
import shapefile #type: ignore
from shapely.geometry import shape, Point   #type: ignore

# Configura el puerto serie  
port = "/dev/ttyAMA0"  
ser = serial.Serial(port, baudrate=9600, timeout=0.1) 

def check(lon, lat):
    # build a shapely point from your geopoint
    point = Point(lon, lat)

    # the contains function does exactly what you want
    return polygon.contains(point)


while True:  
    newdata = ser.readline().decode('utf-8').strip()  
    # read your shapefile
    poly_file='poligono_casona.shp'
    r = shapefile.Reader(poly_file)

    # Verifica si se recibe una sentencia GPRMC  
    if newdata[0:6] == "$GPRMC":  
        newmsg = pynmea2.parse(newdata)  
        status = newmsg.status   
        # Maneja los estados A y V  
        if status == "A":  
            lat = newmsg.latitude  
            lon = newmsg.longitude  
            gps = f"Lat = {lat} Lng = {lon}"  
            print(gps)  
            speed = newmsg.spd_over_grnd  # velocidad en nudos  
            speed_mps = speed * (0.514444)  # convertimos de nudos a m/s  
            print(f"Speed: {speed:.2f} knots / {speed_mps:.2f} m/s")  

            # get the shapes
            shapes = r.shapes()
            inside_zone = False  # Bandera para verificar si está dentro de alguna zona
            for k in range(len(shapes)):
                # build a shapely polygon from your shape
                polygon = shape(shapes[k])    
                zone_def = check(lon, lat)
                if zone_def:
                    zone = k
                    print('El punto corresponde a la zona ' + str(zone+1))
                    inside_zone = True
                    break  # Sal del bucle si se encuentra una zona

            if not inside_zone:
                print("Estas Fuera de Rango")

        elif status == "V":  
            print("Estoy Agarrando Señal, Krnal ...")  
            print("Estoy Cansado, Jefe")