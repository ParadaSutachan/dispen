import serial  
import time  
import pynmea2  

# Configura el puerto serie  
port = "/dev/ttyAMA0"  
ser = serial.Serial(port, baudrate=9600, timeout=0.1)  

while True:  
    newdata = ser.readline().decode('utf-8').strip()  
    
    # Verifica si se recibe una sentencia GPRMC  
    if newdata[0:6] == "$GPRMC":  
        newmsg = pynmea2.parse(newdata)  
        status = newmsg.status   
        # Maneja los estados A y V  
        if status == "A":  
            lat = newmsg.latitude  
            lng = newmsg.longitude  
            gps = f"Lat = {lat} Lng = {lng}"  
            print(gps)  
            speed = newmsg.spd_over_grnd  # velocidad en nudos  
            speed_mps = speed * (0.514444)  # convertimos de nudos a m/s  
            print(f"Speed: {speed:.2f} knots / {speed_mps:.2f} m/s")  
        elif status == "V":  
            print("Estoy Agarrando Señal, Krnal ...")  
            print("Estoy Cansado, Jefe")
	