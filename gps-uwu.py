#!/usr/bin/env python
import pigpio
import time

# Conectar al daemon pigpio
pi = pigpio.pi()

# Verificar que la conexión con el daemon pigpio esté activa
if not pi.connected:
    exit()

# Definir el pin GPIO donde está conectado el ESC
ESC_PIN = 20  # Cambia este valor por el pin GPIO que estés utilizando

# Mover el motor a 1200 microsegundos (velocidad baja)
def move_to_1200():
    print("Moviendo el motor a 1200 us...")
    pi.set_servo_pulsewidth(ESC_PIN, 1200)  # Configurar el PWM a 1200 microsegundos
    time.sleep(5)  # Mantener durante 5 segundos

try:
    move_to_1200()

finally:
    # Detener el PWM
    pi.set_servo_pulsewidth(ESC_PIN, 0)  # Apagar el PWM
    pi.stop()  # Finalizar pigpio
    print("Proceso completado y conexión con pigpio cerrada.")
