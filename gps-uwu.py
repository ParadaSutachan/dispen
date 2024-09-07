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

# Función para iniciar el ESC con señal mínima, luego subir gradualmente
def initialize_and_ramp_up():
    print("Enviando señal mínima de 1000 us para iniciar el ESC...")
    pi.set_servo_pulsewidth(ESC_PIN, 1000)  # Señal mínima de seguridad
    time.sleep(3)  # Mantener durante 3 segundos

    print("Aumentando gradualmente la señal hasta 2000 us...")
    for pwm_value in range(1000, 2001, 100):  # Aumentar de 1000 a 2000 en pasos de 100
        print(f"Enviando PWM de {pwm_value} us...")
        pi.set_servo_pulsewidth(ESC_PIN, pwm_value)
        time.sleep(2)

try:
    initialize_and_ramp_up()

finally:
    pi.set_servo_pulsewidth(ESC_PIN, 0)  # Apagar el PWM al finalizar
    pi.stop()  # Finalizar pigpio
    print("Proceso completado y conexión con pigpio cerrada.")
