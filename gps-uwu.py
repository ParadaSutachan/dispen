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

# Función para recalibrar el ESC
def recalibrate_esc():
    print("Calibrando el ESC...")

    # Enviar señal máxima (2000 us) durante unos segundos
    print("Enviando señal máxima de 2000 us...")
    pi.set_servo_pulsewidth(ESC_PIN, 2000)
    time.sleep(2)

    # Pedir al usuario que desconecte y vuelva a conectar la batería del ESC
    input("Desconecta la batería del ESC ahora y presiona Enter para continuar...")

    # Enviar señal mínima (1000 us) después de reconectar la batería
    print("Enviando señal mínima de 1000 us...")
    pi.set_servo_pulsewidth(ESC_PIN, 1000)
    time.sleep(2)

    print("Calibración completa. Probando señales intermedias...")

    # Probar señal intermedia (1500 us)
    pi.set_servo_pulsewidth(ESC_PIN, 1500)
    time.sleep(2)

    # Probar señal de 1200 us
    pi.set_servo_pulsewidth(ESC_PIN, 1200)
    time.sleep(2)

try:
    recalibrate_esc()

finally:
    pi.set_servo_pulsewidth(ESC_PIN, 0)  # Apagar el PWM
    pi.stop()  # Finalizar pigpio
    print("Proceso completado y conexión con pigpio cerrada.")
