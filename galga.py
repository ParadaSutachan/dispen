# -- coding: utf-8 --
#!/usr/bin/env python3
import time
import pigpio #type: ignore
import math
from pytictoc import TicToc #type: ignore
import numpy as np # type: ignore
from numpy import array  #type: ignore
#import serial #type:ignore

# Inicialización de Pigpio
pi = pigpio.pi()
pi_m = math.pi 

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

K = np.array([[-96.894235555587898, -54.978236188808395, -16.510278445545580, 0.225397518945667]])
Ki = -0.726725918817694
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
set_point_f = 35.0
set_point_w = 25.0
delta_w = 0.0
delta_w_1 = 0.0
delta_f = 0.0
delta_f_1 = 0.0 
delta_f_2 = 0.0

xk = np.array([[0],
               [0],
               [0],
               [0]])

xk1 = np.array([[0], 
                   [0],
                   [0],
                   [0]]) 

# Loop de Control
start_time = time.time()
rk = float(input("Ingrese la referencia:  "))
motor1_speed = 50
# Crear el archivo de salida para guardar los datos
output_file_path = '/home/santiago/Documents/dispensador/dispensador/test_controlador_ss.txt'
with open(output_file_path, 'w') as output_file:
    output_file.write("Tiempo \t PWM \t W \tFlujo \n")

    while(time.time()-start_time <= 20):
        
        t1 = TicToc()       # Tic
        t1.tic()

        #Lectura de Flancos para medir velocidad
        flancos_totales_1 = numero_flancos_A + numero_flancos_B
        FPS = flancos_totales_1 / (600.0)
        W = FPS * ((2 * pi_m) / T)      #Velocidad del motor

        # Soft Sensor
        delta_w = W - set_point_w
        delta_f = 0.1969*delta_w_1 + 1.359*delta_f_1 -0.581*delta_f_2
        fk = delta_f + set_point_f
        ##

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
        ux_k=K@xk
        uk = -uik-float(ux_k[0]) #Accion de Control

        if uk >= 100:

            motor1_speed = 100
        else:
            motor1_speed = abs(uk)
            
        print("uk = " + str(uk))

        control_motor(motor1_pwm_pin, motor1_dir_pin, motor1_speed, 'forward')
        control_motor(motor1_pwm_pin, motor1_dir_pin, 100, 'forward')
        
        delta_f_2 = delta_f_1
        delta_f_1 = delta_f
        xk = xk1
        ek_int_1=ek_int
        ek_1 = ek
        delta_w_1 = delta_w 

        # Registrar los datos en el archivo
        ts = time.time() - start_time
        output_file.write(f"{ts:.2f}\t{uk:.2f}\t{W:.2f}\t{fk:.2f}\n")

        # Restablecer contadores
        numero_flancos_A = 0
        numero_flancos_B = 0
        numero_flancos_A2 = 0
        numero_flancos_B2 = 0

        
        print("Flujo = "+ str(fk))

        e_time = t1.tocvalue()
        toc = abs(T-e_time)         #Toc
        time.sleep(toc)
    

# Deshabilitar motores
pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
pi.set_PWM_dutycycle(motor2_pwm_pin, 0)
pi.write(motor1_en_pin, 0)
pi.write(motor2_en_pin, 0)

# Detener Pigpio
pi.stop()
print('Tiempo de funcionamiento de los motores completado.')