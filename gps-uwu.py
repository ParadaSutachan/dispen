#!/usr/bin/env python
import RPi.GPIO as GPIO  
import time  

# Configuración del pin GPIO  
ESC_PIN = 21  
GPIO.setmode(GPIO.BCM)  
GPIO.setup(ESC_PIN, GPIO.OUT)  

# Configuración de PWM  
pwm = GPIO.PWM(ESC_PIN, 50)  # 50 Hz para controlar el ESC
pwm.start(0)  # Inicializa el PWM con un ciclo de trabajo de 0%  

# Función para convertir de microsegundos a ciclo de trabajo PWM
def set_pwm_from_microseconds(pulse_width):
    # Calcular el duty cycle correspondiente al ancho de pulso en microsegundos
    # 1000 us corresponde a 5% de duty cycle, 2000 us corresponde a 10%
    duty_cycle = (pulse_width / 20000) * 100  # Convertir microsegundos a porcentaje de duty cycle
    pwm.ChangeDutyCycle(duty_cycle)

# Volver a la configuración original: de 1000 a 2000 microsegundos
try:
    print("Volviendo a la configuración original...")

    # Inicialización del ESC al valor mínimo (1000 us)
    set_pwm_from_microseconds(1000)
    time.sleep(3)

    # Subir a un valor de punto neutro (1500 us)
    set_pwm_from_microseconds(1500)
    time.sleep(2)

    # Aumentar gradualmente hasta el valor máximo (2000 us)
    print("Aumentando al máximo...")
    set_pwm_from_microseconds(2000)
    time.sleep(2)

    print("Configuración completa.")
    
finally:
    pwm.stop()  # Detener el PWM
    GPIO.cleanup()  # Limpiar los pines GPIO al finalizar
