#!/usr/bin/env python
import pigpio
import time

# Conectarse al daemon pigpio
pi = pigpio.pi()

# Definir el pin del ESC
ESC_PIN = 21  # Cambia al pin GPIO que estés utilizando

def test_esc():
    print("Probando el ESC...")

    pi.set_servo_pulsewidth(ESC_PIN, 2000)
    print("Señal neutra (2000 us)...")
    time.sleep(1)

    # Enviar señal intermedia (1500 us, punto neutro)
    pi.set_servo_pulsewidth(ESC_PIN, 1045)
    print("Señal neutra (1050 us)...")
    time.sleep(15)



test_esc()

# Apagar el servo PWM y limpiar
pi.set_servo_pulsewidth(ESC_PIN, 0)  # Apagar el PWM
pi.stop()  # Detener pigpio
