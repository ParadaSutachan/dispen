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

# Configuración de pines brushless

ESC_PIN_1 = 21  # Pin GPIO donde está conectado el primer ESC
ESC_PIN_2 = 20  # Pin GPIO donde está conectado el segundo ESC

# Configurar GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PIN_1, GPIO.OUT)
GPIO.setup(ESC_PIN_2, GPIO.OUT)

# Configurar la señal PWM para los ESC (50 Hz)
pwm1 = GPIO.PWM(ESC_PIN_1, 50)  # Frecuencia de 50 Hz (20ms ciclo completo)
pwm2 = GPIO.PWM(ESC_PIN_2, 50)  # Frecuencia de 50 Hz (20ms ciclo completo)

# Inicializar PWM
pwm1.start(0)  # Iniciar con un duty cycle de 0%
pwm2.start(0)  # Iniciar con un duty cycle de 0%

# Configuración de pines de motor y encoder

motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23

# Función para convertir microsegundos a ciclo de trabajo
def set_speed(pwm, pulse_width):
    duty_cycle = pulse_width / 20000 * 100  # Convertir microsegundos a ciclo de trabajo
    pwm.ChangeDutyCycle(duty_cycle)

print("Deteniendo motores...")
set_speed(pwm1, 1000)  # Señal mínima para detener el primer motor
set_speed(pwm2, 1000)

# Limpiar y detener PWM
pwm1.stop()
pwm2.stop()
GPIO.cleanup()

# Deshabilitar motores
pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
pi.set_PWM_dutycycle(motor2_pwm_pin, 0)
pi.write(motor1_en_pin, 0)
pi.write(motor2_en_pin, 0)


# Detener Pigpio
pi.stop()
print('Tiempo de funcionamiento de los motores completado.')
