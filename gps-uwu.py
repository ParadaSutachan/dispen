#!/usr/bin/env python
import pigpio
import time

# Conectar al daemon pigpio
pi = pigpio.pi()

# Verificar que la conexión con el daemon pigpio esté activa
if not pi.connected:
    exit()

# Definir el pin GPIO donde está conectado el ESC
ESC_PIN = 21  # Cambia este valor por el pin GPIO que estés utilizando

# Función para establecer el valor máximo (2000 us)
def set_max_pwm():
    print("Estableciendo el valor máximo de 2000 us...")
    pi.set_servo_pulsewidth(ESC_PIN, 2000)  # Configura el PWM a 2000 us (máximo)
    time.sleep(5)  # Mantener durante 5 segundos para probar el valor máximo

try:
    set_max_pwm()

finally:
    pi.set_servo_pulsewidth(ESC_PIN, 0)  # Apagar el PWM al finalizar
    pi.stop()  # Finalizar pigpio
    print("Proceso completado y conexión con pigpio cerrada.")
