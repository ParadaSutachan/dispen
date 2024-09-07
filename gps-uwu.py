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

# Función para enviar una señal mínima de 1000 us para desbloquear el ESC
def unlock_esc():
    print("Enviando señal mínima de 1000 us para desbloquear el ESC...")
    pi.set_servo_pulsewidth(ESC_PIN, 1000)  # Enviar señal mínima
    time.sleep(3)  # Esperar 3 segundos para que el ESC acepte la señal mínima

# Función para probar valores de PWM desde 1000 us a 1200 us lentamente
def test_motor_slowly():
    for pwm_value in range(1000, 1201, 50):  # Probar con valores 1000, 1050, 1100, 1150, 1200
        print(f"Probando con PWM de {pwm_value} us...")
        pi.set_servo_pulsewidth(ESC_PIN, pwm_value)
        time.sleep(5)  # Mantener durante 3 segundos

try:
    # Enviar señal mínima para desbloquear el ESC
    unlock_esc()

    # Probar incrementos lentos desde 1000 us hasta 1200 us
    test_motor_slowly()

finally:
    pi.set_servo_pulsewidth(ESC_PIN, 0)  # Apagar el PWM
    pi.stop()  # Finalizar pigpio
    print("Proceso completado y conexión con pigpio cerrada.")
