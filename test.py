# -- coding: utf-8 --
#!/usr/bin/env python3
import time
import pigpio
import RPi.GPIO as GPIO
import math
import serial
from pytictoc import TicToc

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
w_n_1= 0.0
fm_n_1= 0.0
fm_n_2= 0.0
W_1 = 0.0
W=0.0

# Variables control maestro esclavo

Kp_m = 0.8824
ki_m = 0.4606
kp_s = 0.08028
ki_s = 0.2746

rk_m= 0.0
yk_m= 0.0
ek_m= 0.0
iek_m= 0.0
iek_m_1= 0.0
upi_m= 0.0

rk_s= 0.0
yk_s= 0.0
ek_s= 0.0
iek_s= 0.0
iek_s_1= 0.0
upi_s= 0.0

setpoint_f= 35.0
setpoint_W = 25.0

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
    duty_cycle = max(0, min(255, duty_cycle))  # Asegurar que duty_cycle esté en el rango 0-255
    pi.set_PWM_dutycycle(pin_pwm, duty_cycle)

    if direction == 'forward':
        pi.write(pin_dir, 1)  # Dirección hacia adelante
    elif direction == 'backward':
        pi.write(pin_dir, 0)  # Dirección hacia atrás
    else:
        raise ValueError("Dirección no válida. Usa 'forward' o 'backward'.")





# Loop de Control
start_time = time.time()
rk_m= float(input("Ingrese la referencia:  "))
# Crear el archivo de salida para guardar los datos
output_file_path = '/home/santiago/Documents/dispensador/dispensador/test_PI_MS.txt'
with open(output_file_path, 'w') as output_file:
    output_file.write("Tiempo \t PWM \t W \tFlujo \n")

    while(time.time()-start_time <= 20):
        
        t1 = TicToc()  
        t1.tic()          # Tic

        #Medicion flujo 
        delta_W = W - setpoint_W
        fm_n= fm_n - setpoint_W
        fm_n= 0.1969*W_1 + 1.359 * fm_n_1 - 0.581*fm_n_2 
        fm_n_2 = fm_n_1
        fm_n_1 = fm_n
        W_1 = delta_W

        fm_n= fm_n + setpoint_f
        print("flujo = "+ str(fm_n))

        #Control esclavo
        rk_s = upi_m
        yk_s = W
        ek_s= rk_s - yk_s
        iek_s = ek_s + iek_s_1
        upi_s = (kp_s*ek_s) + (ki_s*iek_s)
        print("pwm = "+ str(upi_s))

        iek_s_1 = iek_s
        iek_s=0

        #Control maestro
        yk_m = fm_n
        ek_m= rk_m - yk_m
        iek_m = ek_m + iek_m_1
        upi_m = Kp_m*ek_m + ki_m*iek_m

        iek_m_1 = iek_m
        iek_m= 0

        motor1_speed = max(0, min(100, upi_s))  # Asegurar que motor1_speed esté en el rango 0-100
        control_motor(motor1_pwm_pin, motor1_dir_pin, motor1_speed, 'forward')

        flancos_totales_1 = numero_flancos_A + numero_flancos_B
        RPS = flancos_totales_1 / (600.0)
        W = RPS * ((2 * pi_m) / INTERVALO)

        # Registrar los datos en el archivo
        ts = time.time() - start_time
        output_file.write(f"{ts:.2f}\t{upi_s:.2f}\t{W:.2f}\t{fm_n:.2f}")


        # Restablecer contadores
        numero_flancos_A = 0
        numero_flancos_B = 0
        numero_flancos_A2 = 0
        numero_flancos_B2 = 0

        e_time= t1.tocvalue()
        toc= abs(INTERVALO- e_time)
        time.sleep(toc)
        
        
    
# Deshabilitar motores
pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
pi.set_PWM_dutycycle(motor2_pwm_pin, 0)
pi.write(motor1_en_pin, 0)
pi.write(motor2_en_pin, 0)



# Detener Pigpio
pi.stop()
print('Tiempo de funcionamiento de los motores completado.')

