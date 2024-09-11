# -- coding: utf-8 --
#!/usr/bin/env python3

import time  
from encoder import Encoder  
import pigpio

# Inicialización de Pigpio
pi = pigpio.pi()

# Pines de motor y encoder
motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23

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
    
# Configura los pines GPIO para el encoder  
encoder = Encoder(17, 18)  # Cambia los números de los pines según tu conexión  

try:  
    while True:  
        # Leer el valor del encoder  
        pi.write(motor1_en_pin, 1)
        pi.write(motor2_en_pin, 1)

        control_motor(motor1_pwm_pin, motor1_dir_pin, 100, 'forward')

        value = encoder.position  
        print("Posición del encoder:", value)  
        time.sleep(0.2)  

except KeyboardInterrupt:  
    print("Deteniendo...")