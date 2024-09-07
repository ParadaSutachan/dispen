#!/usr/bin/env python
import RPi.GPIO as GPIO  
import time  

# Configuración del pin GPIO  
ESC_PIN = 21  
GPIO.setmode(GPIO.BCM)  
GPIO.setup(ESC_PIN, GPIO.OUT)  

# Configuración de PWM  
pwm = GPIO.PWM(ESC_PIN, 50)  # 50 Hz para el ESC
pwm.start(0)  # Inicializa el PWM con un ciclo de trabajo de 0%  

# Función para convertir de microsegundos a ciclo de trabajo PWM
def set_pwm_from_microseconds(pulse_width):
    # Calcular el duty cycle correspondiente al ancho de pulso en microsegundos
    # 1000 us corresponde a 5% de duty cycle, 2000 us corresponde a 10%
    duty_cycle = (pulse_width / 20000) * 100  # Convertir microsegundos a porcentaje de duty cycle
    pwm.ChangeDutyCycle(duty_cycle)

try:
    print("Calibrando el ESC...")

    # Enviar señal máxima primero (2000 us) durante unos segundos
    set_pwm_from_microseconds(2000)
    print("Manteniendo máximo...")
    time.sleep(3)

    # Luego enviar la señal mínima (1000 us) para que el ESC calibre el rango
    set_pwm_from_microseconds(1000)
    print("Manteniendo mínimo...")
    time.sleep(3)

    print("Calibración completa. Probando valores intermedios...")

    # Probar el valor intermedio (1500 us)
    set_pwm_from_microseconds(1500)
    time.sleep(3)

    # Probar el valor máximo nuevamente
    set_pwm_from_microseconds(2000)
    time.sleep(3)

finally:
    pwm.stop()  # Detener el PWM
    GPIO.cleanup()  # Limpiar los pines GPIO al finalizar
