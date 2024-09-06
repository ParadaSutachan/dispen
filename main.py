#!/usr/bin/env python
import pigpio
import time

# Conectar al demonio pigpio
pi = pigpio.pi()

# Definir el pin GPIO donde está conectado el ESC
ESC_PIN = 21  # Cambia al pin que estés utilizando

# Función para enviar el pulso PWM
def set_pwm(pulse_width):
    pi.set_servo_pulsewidth(ESC_PIN, pulse_width)

# Inicializar el ESC
def initialize_esc():
    print("Inicializando ESC...")
    set_pwm(1100)  # Establece el valor mínimo
    time.sleep(3)

# Ajustar el rango para que 1200 sea más manejable
def run_motor_at_custom_speed():
    print("Probando con PWM ajustado...")
    
    # Prueba con un valor ajustado de 1150 (entre 1100 y 1200) para disminuir la velocidad
    set_pwm(1150)
    time.sleep(3)
    
    # O prueba diferentes valores para reducir la velocidad a niveles más bajos
    set_pwm(1180)
    time.sleep(3)
    
    # Finalmente, prueba en 1200
    set_pwm(1200)
    time.sleep(3)

try:
    initialize_esc()
    run_motor_at_custom_speed()

finally:
    pi.set_servo_pulsewidth(ESC_PIN, 0)  # Apagar el PWM
    pi.stop()  # Finalizar pigpio
