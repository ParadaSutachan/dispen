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

# Función para mover el motor a un valor neutro de 1500 us
def test_motor_neutral():
    print("Enviando señal neutra de 1500 us...")
    pi.set_servo_pulsewidth(ESC_PIN, 1500)  # Enviar señal neutra (generalmente punto de arranque)
    time.sleep(5)  # Mantener durante 5 segundos

try:
    test_motor_neutral()

finally:
    pi.set_servo_pulsewidth(ESC_PIN, 0)  # Apagar el PWM
    pi.stop()  # Finalizar pigpio
    print("Proceso completado y conexión con pigpio cerrada.")
