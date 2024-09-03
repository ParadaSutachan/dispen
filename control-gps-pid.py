# -- coding: utf-8 --
#!/usr/bin/env python3

import serial
import time  
import pynmea2
import shapefile
from shapely.geometry import shape, Point
import pigpio
import RPi.GPIO as GPIO
import math
from pytictoc import TicToc

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

INTERVALO = 0.2  # Intervalo de tiempo en segundos para cálculo de RPM

# Contadores de flancos
numero_flancos_A = 0
numero_flancos_B = 0
numero_flancos_A2 = 0
numero_flancos_B2 = 0

# Variables para RPM y RPS
RPS = 0.0
RPM = 0.0
RPS2 = 0.0
RPM2 = 0.0

# Variables de flujo
fm_n= 0.0
fm_n2= 0.0
W=0.0
W2= 0.0

# Variables control maestro esclavo
Kp_m = 0.049398
ki_m = 0.241
kp_s = 0.36612
ki_s = 1.097

Kp_m2 = 0.049398
ki_m2 = 0.241
kp_s2 = 0.36612
ki_s2 = 1.097

rk_m= 0.0
yk_m= 0.0
ek_m= 0.0
iek_m= 0.0
iek_m_1= 0.0    # PARA M1
upi_m= 0.0
up_m = 0.0
ui_m = 0.0

rk_m2= 0.0
yk_m2= 0.0
ek_m2= 0.0
iek_m2= 0.0     # PARA M2
iek_m_12= 0.0
upi_m2= 0.0
up_m2 = 0.0
ui_m2 = 0.0

rk_s= 0.0
yk_s= 0.0
ek_s= 0.0
iek_s= 0.0
iek_s_1= 0.0    # PARA M1
upi_s= 0.0
up_s = 0.0
ui_s = 0.0
k= 0.0

rk_s2= 0.0
yk_s2= 0.0
ek_s2= 0.0
iek_s2= 0.0        # PARA M2
iek_s_12= 0.0   
upi_s2= 0.0
up_s2 = 0.0
ui_s2 = 0.0
k= 0.0
k2=0.0

setpoint_f= 21.3465
setpoint_W = 28
delta_fn= 0.0
delta_fn_2= 0.0
ref= 12.0
ref2=10.0
ancho_faja= 3.0
velocidad =0.0

# Variable Voltaje
v1 = 0.0
v2 = 0.0

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



# Habilitar motores
pi.write(motor1_en_pin, 1)
pi.write(motor2_en_pin, 1)

# Loop principal de control
start_time = time.time()

rk_m = float(ref * ancho_faja * velocidad)
rk_m2 = float(ref2 * ancho_faja * velocidad)  #  M2

delta_W_1 = 0.0
delta_fn_1 = 0.0
delta_fn_2 = 0.0

delta_W_12 = 0.0
delta_fn_12 = 0.0
delta_fn_22 = 0.0

# Crear el archivo de salida para guardar los datos
output_file_path = '/home/santiago/Documents/dispensador/dispen/maestro_esclavo_definitivo.txt'
with open(output_file_path, 'w') as output_file:
    output_file.write("Tiempo \t PWM_1 \t PWM_M2 \t W1 \t W2 \tFlujo \t Flujo2\n")

    while time.time()-start_time <= 80:
        # Lee los datos del GPS
        newdata = ser.readline().decode('utf-8').strip()
        # Leer el archivo shapefile
        poly_file = 'poligono_casona.shp'
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
                velocidad = speed * 0.514444  # convertimos de nudos a m/s  
                print(f"Speed: {speed:.2f} knots / {velocidad:.2f} m/s")  

                # get the shapes
                shapes = r.shapes()
                for k in range(len(shapes)):
                    # build a shapely polygon from your shape
                    polygon = shape(shapes[k])    
                    zone_def = check(lon, lat)
                    if zone_def:
                        zone = k
                        print('El punto corresponde a la zona ' + str(zone+1))
                    else:
                        print("Estas Fuera de Rango")
            elif status == "V":  
                print("Estoy Agarrando Señal, Krnal ...")  
                print("Estoy Cansado, Jefe")

        t1 = TicToc()  
        t1.tic()   
        k += 1       # Tic
        k2 += 1

        flancos_totales_1 = numero_flancos_A + numero_flancos_B
        RPS = flancos_totales_1 / 600.0
        W = RPS * ((2 * pi_m) / INTERVALO)
        print("Velocidad: " + str(W))

        flancos_totales_2 = numero_flancos_A2 + numero_flancos_B2
        RPS2 = flancos_totales_2 / 600.0
        W2 = RPS2 * ((2 * pi_m) / INTERVALO)
        print("Velocidad M2: " + str(W2))

        delta_fn = flancos_totales_1
        delta_W = W

        delta_fn2 = flancos_totales_2
        delta_W2 = W2

        fm_n = flancos_totales_1 + 180
        fm_n2 = flancos_totales_2 + 160
        rk_m = 20
        rk_m2 = 15

        ek_m = fm_n - W                   # Error de velocidad
        up_m = ek_m * Kp_m
        iek_m = iek_m_1 + ek_m            # Error integral
        ui_m = ki_m * iek_m * INTERVALO
        upi_m = up_m + ui_m

        ek_m2 = fm_n2 - W2                # Error de velocidad
        up_m2 = ek_m2 * Kp_m2
        iek_m2 = iek_m2 + ek_m2           # Error integral
        ui_m2 = ki_m2 * iek_m2 * INTERVALO
        upi_m2 = up_m2 + ui_m2

        ek_s = rk_s - W                   # Error de velocidad
        up_s = ek_s * kp_s
        iek_s = iek_s_1 + ek_s            # Error integral
        ui_s = ki_s * iek_s * INTERVALO
        upi_s = up_s + ui_s

        ek_s2 = rk_s2 - W2                # Error de velocidad
        up_s2 = ek_s2 * kp_s2
        iek_s2 = iek_s2 + ek_s2           # Error integral
        ui_s2 = ki_s2 * iek_s2 * INTERVALO
        upi_s2 = up_s2 + ui_s2

        v1 = upi_s + upi_m
        v2 = upi_s2 + upi_m2

        print("V1: " + str(v1))
        print("V2: " + str(v2))

        if v1 <= 0:
            v1 = 0
        elif v1 > 48:
            v1 = 48
        if v2 <= 0:
            v2 = 0
        elif v2 > 48:
            v2 = 48

        # Control del motor 1
        speed_percent_m1 = v1 * 100 / 48  # Convierte v1 a porcentaje
        control_motor(motor1_pwm_pin, motor1_dir_pin, speed_percent_m1, 'forward')

        # Control del motor 2
        speed_percent_m2 = v2 * 100 / 48  # Convierte v2 a porcentaje
        control_motor(motor2_pwm_pin, motor2_dir_pin, speed_percent_m2, 'forward')

        # Actualiza valores anteriores
        delta_fn_2 = delta_fn_1
        delta_fn_1 = delta_fn
        delta_W_1 = delta_W

        delta_fn_22 = delta_fn_12
        delta_fn_12 = delta_fn2
        delta_W_12 = delta_W2

        iek_m_1 = iek_m
        iek_s_1 = iek_s

        t1.toc()
        time.sleep(0.2)

        # Guardar en el archivo de salida
        output_file.write(f"{time.time() - start_time} \t {v1} \t {v2} \t {W} \t {W2} \t {fm_n} \t {fm_n2}\n")

# Deshabilitar motores
pi.write(motor1_en_pin, 0)
pi.write(motor2_en_pin, 0)
