#!/usr/bin/env python
import pigpio
import time

# Conectar al daemon pigpio
pi = pigpio.pi()


# Definir el pin GPIO donde está conectado el ESC
ESC_PIN = 21  # Cambia este valor por el pin GPIO que estés utilizando

print("Estableciendo el valor máximo de 2000 us...")
pi.set_servo_pulsewidth(ESC_PIN, 2000)  # Configura el PWM a 2000 us (máximo)
time.sleep(10)  # Mantener durante 5 segundos para probar el valor máximo



pi.set_servo_pulsewidth(ESC_PIN, 0)  # Apagar el PWM al finalizar
pi.stop()  # Finalizar pigpio
print("Proceso completado y conexión con pigpio cerrada.")
