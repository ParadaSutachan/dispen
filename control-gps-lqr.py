# -- coding: utf-8 --
#!/usr/bin/env python3
import time
import pigpio #type: ignore
import math
from pytictoc import TicToc #type: ignore
import numpy as np # type: ignore
from numpy import array  #type: ignore
import serial #type:ignore
import RPi.GPIO as GPIO #type:ignore
import pynmea2 #type: ignore
import shapefile #type: ignore
from shapely.geometry import shape, Point   #type: ignore

# Inicialización de Pigpio
pi = pigpio.pi()
pi_m = math.pi 

# Configura el puerto serie  
port = "/dev/ttyAMA0"  
ser = serial.Serial(port, baudrate=9600, timeout=0.1)  

# Configuración de pines de motor y encoder

motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23

PIN_ENCODER_A = 18
PIN_ENCODER_B = 17
PIN_ENCODER2_A = 16
PIN_ENCODER2_B = 19

T = 0.2

# Configuración de pines de entrada para los encoders
pi.set_mode(PIN_ENCODER_A, pigpio.INPUT)
pi.set_pull_up_down(PIN_ENCODER_A, pigpio.PUD_UP)
pi.set_mode(PIN_ENCODER_B, pigpio.INPUT)
pi.set_pull_up_down(PIN_ENCODER_B, pigpio.PUD_UP)

pi.set_mode(PIN_ENCODER2_A, pigpio.INPUT)
pi.set_pull_up_down(PIN_ENCODER2_A, pigpio.PUD_UP)
pi.set_mode(PIN_ENCODER2_B, pigpio.INPUT)
pi.set_pull_up_down(PIN_ENCODER2_B, pigpio.PUD_UP)

# Funciones de callback para contar flancos
def contador_flancos_encoder(gpio, level, tick):
    global numero_flancos_A
    numero_flancos_A += 1

def contador_flancos_encoder_b(gpio, level, tick):
    global numero_flancos_B
    numero_flancos_B += 1

def contador_flancos_encoder2(gpio, level, tick):
    global numero_flancos_A2
    numero_flancos_A2 += 1

def contador_flancos_encoder_b2(gpio, level, tick):
    global numero_flancos_B2
    numero_flancos_B2 += 1

# Configuración de callbacks
cb1 = pi.callback(PIN_ENCODER_A, pigpio.EITHER_EDGE, contador_flancos_encoder)
cb3 = pi.callback(PIN_ENCODER2_A, pigpio.EITHER_EDGE, contador_flancos_encoder2)

# Función para controlar el motor
def control_motor(pin_pwm, pin_dir, speed_percent, direction):
    duty_cycle = int(speed_percent * 255 / 100)
    pi.set_PWM_dutycycle(pin_pwm, duty_cycle)

    if direction == 'forward':
        pi.write(pin_dir, 1)  # Dirección hacia adelante
    elif direction == 'backward':
        pi.write(pin_dir, 0)  # Dirección hacia atrás
    else:
        raise ValueError("Dirección no válida. Usa 'forward' o 'backward'.")

def check(lon, lat):
    # build a shapely point from your geopoint
    point = Point(lon, lat)

    # the contains function does exactly what you want
    return polygon.contains(point)

# Contadores de flancos
numero_flancos_A = 0
numero_flancos_B = 0
numero_flancos_A2 = 0
numero_flancos_B2 = 0

# Matrices A,B,C,D del modelo

A = np.array([[0.894130801660748, 0.145973156319373, 0.066225686842812, 0.067086287188154],
            [-0.441836100194477, 0.748393868632439, 0.027872957540047, -0.026971019471969],
            [0.035223234234442, -0.404264770036147, 0.863341393359765, -0.539775953712378],
            [0.173570457083066, 0.507384555424424, 0.507384555424424, 0.507384555424424]])

B = np.array([[-0.000419785220888],
              [-0.006447149525435],
              [-0.003587244670196],
              [0.020003639959133]])

C = np.array([[-46.886951437617398, -8.778687090880192, 5.312400689483614, -2.660688977805191]])

D = 0 

# Matriz K y L y constante Ki 

#K = np.array([[-96.894235555587898, -54.978236188808395, -16.510278445545580, 0.225397518945667]])
# la que tenia al inicio K = np.array([[ -28.943823250907375,-14.916476691659597,  -5.284958871641084,  0.606037301182901]])
#K = np.array([[ -46.687992146378335, -25.234466885110013,  -8.383515963162855,   0.735588807482883]])
#K = np.array([[ -39.182988561814568, -20.829192398061014,  -7.083573262725010,   0.700789477478013]]) #Mejor respuesta segun Santiago
#K = np.array([[ -41.629745839451004, -22.260482684956859,  -7.509364454424423,   0.715540695413975 ]])
K = np.array([[ -40.836100468904952, -21.795633546054280,  -7.371481681852714,   0.711152516724570 ]])

#Ki = -0.344608368322768 # no puede ser mayor
#Ki = -0.286889068064602  #best response
#Ki = -0.305710358015974
Ki= -0.299605441621511


L = np.array([[-0.001633156413536],
              [0.000304322683582],
              [0.002212041259930],
              [-0.000008608682074]])

Ao = (A-L@C)
Bo = np.array([[-0.000419785220888, -0.001633156413536],
              [-0.006447149525435, 0.000304322683582],
              [-0.003587244670196, 0.002212041259930],
              [0.020003639959133, -0.000008608682074]])
#Definición de ek, rk, flujo y X

ek = 0.0
ek_1 = 0.0
ek_int = 0.0
ek_int_1 = 0.0
rk = 0.0
fk = 0.0
W = 0.0
uk = 0.0
F_b = 21.3465
W_b = 28
delta_w = 0.0
delta_w_1 = 0.0
delta_f = 0.0
delta_f_1 = 0.0 
delta_f_2 = 0.0
k=0
g=0
rate = 0.0
faja = 3.0

xk = np.array([[0],
               [0],
               [0],
               [0]])

xk1 = np.array([[0], 
                   [0],
                   [0],
                   [0]]) 

#Inicializacion del arduino

while True:

    newdata = ser.readline().decode('utf-8').strip()  
    
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
            break

        elif status == "V":  
            print("")

# Loop de Control

# Habilitar motores
pi.write(motor1_en_pin, 1)
pi.write(motor2_en_pin, 1)

# Crear el archivo de salida para guardar los datos
output_file_path = '/home/santiago/Documents/dispensador/dispen/beta-test.txt'

with open(output_file_path, 'w') as output_file:
    output_file.write("Tiempo \t PWM \t W \t Referencia \t Flujo \n")
    start_time = time.time()

    while(True):
        
        t1 = TicToc()       # Tic
        t1.tic()
        k += 1
        g +=1

        if  g == 4:
            newdata = ser.readline().decode('utf-8').strip()    
             # Verifica si se recibe una sentencia GPRMC  
            if newdata[0:6] == "$GPRMC":  
                newmsg = pynmea2.parse(newdata)  
                status = newmsg.status
                # read your shapefile
                poly_file='poligono_casona.shp'
                r = shapefile.Reader(poly_file)  
                # Maneja los estados A y V  
                if status == "A":  
                    lat = newmsg.latitude  
                    lon = newmsg.longitude  
                    gps = f"Lat = {lat} Lng = {lon}"  
                    print(gps)  
                    speed = newmsg.spd_over_grnd  # velocidad en nudos  
                    speed_mps = speed * (0.514444)  # convertimos de nudos a m/s  
                    print(f"Speed: {speed_mps:.2f} m/s")
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
                            zona = zone+1
                            inside_zone = True
                            if zona == 1:
                                rate = 15
                            elif zona == 2:
                                rate = 8
                            g=0
                            break  # Sal del bucle si se encuentra una zona

                    while not inside_zone:
                        print("Estas Fuera de Rango . . .")
                        rk = 0.0
                        control_motor(motor1_pwm_pin, motor1_dir_pin, 0, 'forward')


        #Lectura de Flancos para medir velocidad
        flancos_totales_1 = numero_flancos_A + numero_flancos_B
        FPS = flancos_totales_1 / (600.0)
        W = FPS * ((2 * pi_m) / T)      #Velocidad del motor
        print("Velocidad: " + str(W))


        # Soft Sensor
        delta_w = W-W_b
        delta_f = 0.1969*delta_w_1 + 1.359*delta_f_1 -0.581*delta_f_2

        if k <= 3:
             fk= 0
        else :
             fk=delta_f+F_b
        # Calulo de la referencia #

        float(speed_mps)

        if speed_mps <= 0.3:
            speed_mps =0.0
        rk = speed_mps*rate*faja

        ##Observador
        uo = np.array([[uk],
                       [fk]])
        
        xk1 = Ao@xk + Bo@uo
        ##

        ## Controlador
        ek = rk - fk
        print("ek: "+str(ek))
        ek_int = ek_1 + ek_int_1
        uik = ek_int*Ki
        ux_k= K@xk
        uk = -uik-float(ux_k[0]) #Accion de Control
        if uk < 0 or uk > 100:
            if uk < 0 :
                uik = 0 - float(ux_k[0])
            if uk >100:
                uik = -100 - float(ux_k[0])

        uk = -uik-float(ux_k[0])        
        motor1_speed = uk  
        print("uk = " + str(uk))

        control_motor(motor1_pwm_pin, motor1_dir_pin, motor1_speed, 'forward')
        
        delta_f_2 = delta_f_1
        delta_f_1 = delta_f
        xk = xk1
        ek_int_1=ek_int
        ek_1 = ek
        delta_w_1 = delta_w 

        # Registrar los datos en el archivo
        ts = time.time() - start_time
        output_file.write(f"{ts:.2f}\t{uk:.2f}\t{W:.2f}\t{rk} \t{fk:.2f}\n)
        output_file.flush()

        # Restablecer contadores
        numero_flancos_A = 0
        numero_flancos_B = 0
        numero_flancos_A2 = 0
        numero_flancos_B2 = 0

        print("Flujo = "+ str(fk))

        e_time = t1.tocvalue()
        toc = T-e_time         #Toc
        time.sleep(toc)
    

# Deshabilitar motores
pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
pi.set_PWM_dutycycle(motor2_pwm_pin, 0)
pi.write(motor1_en_pin, 0)
pi.write(motor2_en_pin, 0)


# Detener Pigpio
pi.stop()
print('Tiempo de funcionamiento de los motores completado.')
