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

t1 = TicToc()
t2 = TicToc()
salto_linea = 0

arduino_port = '/dev/ttyACM0'  # Puerto donde está conectada la placa Arduino
arduino_baud = 9600

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

# Variables globales para la galga
peso_actual = 0.0
GPIO.setwarnings(False)  # Eliminar los warnings
arduino = serial.Serial(arduino_port, arduino_baud)
time.sleep(10)  # Esperar a que la conexión serial se establezca

def esperar_inicializacion_arduino():
    while True:
        if arduino.in_waiting > 0:
            mensaje = arduino.readline().decode('utf-8').strip()
            if mensaje == "Listo para pesar":
                print("Arduino ha completado la inicialización.")
                break
            else:
                print(f"Mensaje de Arduino: {mensaje}")

def control_motores_y_medicion():
    global numero_flancos_A, numero_flancos_B, numero_flancos_A2, numero_flancos_B2, RPM, RPM2, RPS, RPS2, peso_actual, v1, v2, salto_linea
    
    # Habilitar motores
    pi.write(motor1_en_pin, 1)
    pi.write(motor2_en_pin, 1)

    # Ruta del archivo
    file_path = '/home/santiago/Documents/dispensador/dispensador/Pbrs1.txt'
    
    # Lectura de archivo
    with open(file_path, 'r') as file:
        lines = file.readlines()
        total_lines = len(lines)
        current_line1 = 0
        current_line2 = 0
        
        peso_actual = arduino.readline().decode('utf-8')
        print(peso_actual)

        start_time = time.time()

        # Crear el archivo de salida para guardar los datos
        output_file_path = '/home/santiago/Documents/dispensador/dispensador/M2_Rojo.txt'
        with open(output_file_path, 'w') as output_file:
            output_file.write("Tiempo\t PWM \t Velocidad Angular\t RPM \tPeso (g)\n")

            # Bucle principal
            print('Iniciando la medición y control de los motores.')

            while time.time() - start_time <= 125 :  # Ejecutar durante 120 segundos
                loop_start_time = time.time()
                
                # Obtener velocidades de los motores
                line1 = lines[current_line1].strip()
                line2 = lines[current_line2].strip()
                motor1_speed = int(line1)
                motor2_speed = int(line2)

                # Controlar los motores con las velocidades especificadas
                control_motor(motor1_pwm_pin, motor1_dir_pin, motor1_speed, 'forward')
                control_motor(motor2_pwm_pin, motor2_dir_pin, motor2_speed, 'forward')

                # Avanzar en las líneas circularmente
                if salto_linea == 7:
                    current_line1 = (current_line1 + 1) % total_lines   
                    current_line2 = (current_line2 + 1) % total_lines
                    salto_linea = 0

                # Medir peso
                if arduino.in_waiting > 0:
                    peso_actual = arduino.readline().decode('utf-8')
                    print(peso_actual)
                
                # Calcular voltajes
                v1 = (0.0867 * motor1_speed) + 0.00898
                v2 = (0.0866 * motor2_speed) + 0.00967

                # Calcular RPM para el motor 1
                flancos_totales_1 = numero_flancos_A + numero_flancos_B
                RPS = flancos_totales_1 / (600.0)
                W = RPS * ((2 * pi_m) / INTERVALO)
                RPM = W * (30 / pi_m)

                # Calcular RPM para el motor 2
                flancos_totales_2 = numero_flancos_A2 + numero_flancos_B2
                RPS2 = flancos_totales_2 / (600.0)
                W2 = RPS2 * ((2 * pi_m) / INTERVALO)
                RPM2 = W2 * (30 / pi_m)

                # Registrar los datos en el archivo
                t = time.time() - start_time
                output_file.write(f"{t:.2f}\t{motor2_speed}\t{W2:.2f}\t{RPM2:.2f}\t{peso_actual}")
                # output_file.write("\n")
                output_file.flush()  # Asegurarse de guardar los datos

                # Restablecer contadores
                numero_flancos_A = 0
                numero_flancos_B = 0
                numero_flancos_A2 = 0
                numero_flancos_B2 = 0
                salto_linea += 1
                
                # Controlar el tiempo de muestreo
                elapsed_time = time.time() - loop_start_time
                sleep_time = INTERVALO - elapsed_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

            # Deshabilitar motores
            pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
            pi.set_PWM_dutycycle(motor2_pwm_pin, 0)
            pi.write(motor1_en_pin, 0)
            pi.write(motor2_en_pin, 0)

            # Detener Pigpio
            pi.stop()
            print('Tiempo de funcionamiento de los motores completado.')

# Esperar a que Arduino complete la inicialización
esperar_inicializacion_arduino()

# Ejecutar el control de motores y medición
control_motores_y_medicion()
