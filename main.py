#!/usr/bin/env python

# esc_control.py
# Controla el throttle del ESC en porcentaje (0% a 100%)
# Asegúrate de que pigpiod esté en ejecución con 'sudo pigpiod'

import time
import pigpio

SERVO = 21  # Ajusta este valor según el pin que estés usando
MIN_THROTTLE_US = 1000  # Ancho de pulso en microsegundos para el mínimo throttle
MAX_THROTTLE_US = 2000  # Ancho de pulso en microsegundos para el máximo throttle

# Inicialización de Pigpio
pi = pigpio.pi()  # Conectar con el Pi local

def set_throttle_percentage(percentage):
    """Configura el throttle del ESC basado en un porcentaje (0 a 100%)."""
    if percentage < 0 or percentage > 100:
        raise ValueError("El porcentaje debe estar entre 0 y 100.")

    # Convertir el porcentaje en un ancho de pulso PWM
    pulse_width = MIN_THROTTLE_US + (MAX_THROTTLE_US - MIN_THROTTLE_US) * (percentage / 100.0)
    
    pi.set_servo_pulsewidth(SERVO, pulse_width)

try:
    # Inicializar el ESC
    print("Inicializando el ESC...")
    set_throttle_percentage(0)  # Configurar el throttle al mínimo
    time.sleep(10)  # Esperar a que el ESC se inicialice
    
    # Controlar el ESC
    print("Configurando el throttle al 50%...")
    set_throttle_percentage(50)  # Configurar el throttle al 50%
    time.sleep(10)  # Mantener el throttle durante 10 segundos
    
    print("Configurando el throttle al 100%...")
    set_throttle_percentage(100)  # Configurar el throttle al 100%
    time.sleep(10)  # Mantener el throttle durante 10 segundos
    
finally:
    # Detener el ESC y desconectar el Pi
    pi.set_servo_pulsewidth(SERVO, 0)  # Apagar el servo
    pi.stop()  # Desconectar del Pi
    print("Desconectado del Pi.")
