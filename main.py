#!/usr/bin/env python
import pigpio
import time

# Conectar al daemon pigpio
pi = pigpio.pi()

# Definir el pin GPIO donde está conectado el ESC
ESC_PIN = 21  # Cambia al pin GPIO que estés utilizando

# Función para enviar el pulso PWM
def set_pwm(pulse_width):
    pi.set_servo_pulsewidth(ESC_PIN, pulse_width)

# Inicializar el ESC en el valor máximo (2000 us)
def calibrate_esc():
    print("Iniciando recalibración del ESC...")

    # Enviar el valor máximo (2000 us) durante unos segundos
    set_pwm(2000)
    print("Manteniendo señal máxima (2000 us)...")
    time.sleep(2)

    # Desconectar la batería del ESC durante unos segundos, luego reconectar
    input("Desconecta la batería del ESC ahora y presiona Enter para continuar...")

    # Enviar el valor mínimo (1000 us) una vez reconectada la batería
    set_pwm(1000)
    print("Manteniendo señal mínima (1000 us)...")
    time.sleep(2)

    print("Recalibración completa. El ESC está listo para operar.")

# Configurar el ESC a un valor intermedio (1500 us)
def run_motor_neutral():
    print("Probando el motor a velocidad neutral (1500 us)...")

    # Enviar el valor de 1500 microsegundos para probar la velocidad neutra
    set_pwm(1500)
    time.sleep(5)  # Mantener durante 5 segundos

try:
    # Recalibrar el ESC para volver al rango estándar
    calibrate_esc()

    # Probar el moto
