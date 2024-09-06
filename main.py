#!/usr/bin/env python
import pigpio
import time

# Conectarse al daemon pigpio
pi = pigpio.pi()

# Definir el pin GPIO donde está conectado el ESC
ESC_PIN = 21  # Cambia al pin GPIO que estés utilizando

# Función para enviar el pulso PWM
def set_pwm(pulse_width):
    pi.set_servo_pulsewidth(ESC_PIN, pulse_width)

# Inicializar el ESC en el valor máximo (2500 us)
def calibrate_esc():
    print("Iniciando calibración del ESC...")

    # Enviar el valor máximo (2500 us) durante unos segundos
    set_pwm(2500)
    print("Manteniendo señal máxima (2500 us)...")
    time.sleep(2)

    # Desconectar la batería del ESC durante unos segundos, luego reconectar
    input("Desconecta la batería del ESC ahora y presiona Enter para continuar...")

    # Enviar el valor mínimo (1000 us) una vez reconectada la batería
    set_pwm(1000)
    print("Manteniendo señal mínima (1000 us)...")
    time.sleep(2)

    print("Calibración completa. El ESC está listo para operar.")

# Configurar el ESC al valor máximo de 2500 us
def run_motor_max():
    print("Configurando el ESC a su valor máximo (2500 us)...")

    # Enviar el valor máximo de 2500 microsegundos
    set_pwm(2500)
    time.sleep(5)  # Mantener durante 5 segundos

try:
    # Inicializar la calibración del ESC
    calibrate_esc()

    # Ejecutar el motor a 2500 microsegundos
    run_motor_max()

finally:
    # Detener el PWM
    set_pwm(0)
    pi.stop()
    print("Proceso completado y conexión con pigpio cerrada.")
