#!/usr/bin/env python
import pigpio
import time

# Conectarse al daemon pigpio
pi = pigpio.pi()

# Definir el pin del ESC
ESC_PIN = 21  # Cambia al pin GPIO que estés utilizando

# Enviar señal mínima de 1000 microsegundos para el límite inferior
def calibrate_min():
    print("Enviando señal mínima para calibración...")
    pi.set_servo_pulsewidth(ESC_PIN, 1100)  # 1000us = señal mínima
    time.sleep(2)  # Esperar 2 segundos para que el ESC registre el mínimo

calibrate_min()

# Nota: Después de esto, el ESC debería estar calibrado para aceptar señales de 1000 a 2000 microsegundos.
