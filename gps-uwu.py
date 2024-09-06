#!/usr/bin/env python
import pigpio
import time

# Conectarse al daemon pigpio
pi = pigpio.pi()

# Definir el pin del ESC
ESC_PIN = 21  # Cambia al pin GPIO que estés utilizando

# Enviar señal máxima de 2000 microsegundos para el límite superior
def calibrate_max():
    print("Enviando señal máxima para calibración...")
    pi.set_servo_pulsewidth(ESC_PIN, 2000)  # 2000us = señal máxima
    time.sleep(2)  # Esperar 2 segundos para que el ESC registre el máximo

<<<<<<< HEAD
calibrate_max()

# Nota: Mantén encendido durante este tiempo y desconecta la batería del ESC después de 2 segundos.
=======
# Volver a la configuración original: de 1000 a 2000 microsegundos
try:
    set_pwm_from_microseconds(1600)
    time.sleep(2)

    print("Configuración completa.")
    
finally:
    pwm.stop()  # Detener el PWM
    GPIO.cleanup()  # Limpiar los pines GPIO al finalizar
>>>>>>> 41b0b008625864cba1929379a062040c27cd4ff6
