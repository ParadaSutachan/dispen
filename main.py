#!/usr/bin/env python
import pigpio
import time

# Conectarse al daemon pigpio
pi = pigpio.pi()

# Definir el pin del ESC
ESC_PIN = 21  # Cambia al pin GPIO que estés utilizando

def test_esc():
    print("Probando el ESC...")

    # Enviar señal intermedia (1500 us, punto neutro)
    pi.set_servo_pulsewidth(ESC_PIN, 1500)
    print("Señal neutra (1500 us)...")
    time.sleep(10)

    # Enviar señal máxima (2000 us)
    pi.set_servo_pulsewidth(ESC_PIN, 1050)
    print("Señal máxima (2000 us)...")
    time.sleep(10)

    # Detener motor con señal mínima (1000 us)
    pi.set_servo_pulsewidth(ESC_PIN, 1100)
    print("Señal mínima (1000 us)...")
    time.sleep(10)

test_esc()

# Apagar el servo PWM y limpiar
pi.set_servo_pulsewidth(ESC_PIN, 0)  # Apagar el PWM
pi.stop()  # Detener pigpio
