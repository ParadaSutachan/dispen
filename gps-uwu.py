#!/usr/bin/env python
import pigpio
import time

# Conectarse al daemon pigpio
pi = pigpio.pi()

# Definir el pin del ESC
ESC_PIN = 21  # Cambia al pin GPIO que estés utilizando
ESC_PIN2 = 20  # Cambia al pin GPIO que estés utilizando

# Enviar señal máxima de 2000 microsegundos para el límite superior

print("Enviando señal máxima para calibración...")
pi.set_servo_pulsewidth(ESC_PIN, 6)  # 2000us = señal máxima
time.sleep(5)  # Esperar 2 segundos para que el ESC registre el máximo

print("Enviando señal 1200 para calibración...")
pi.set_servo_pulsewidth(ESC_PIN2, 10)  # 2000us = señal máxima
time.sleep(5)  # Esperar 2 segundos para que el ESC registre el máximo


pi.set_servo_pulsewidth(ESC_PIN, 0)  # 2000us = señal máxima
pi.set_servo_pulsewidth(ESC_PIN2, 0)  # 2000us = señal máxima
pi.stop()
# Nota: Mantén encendido durante este tiempo y desconecta la batería del ESC después de 2 segundos.
