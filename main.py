#!/usr/bin/env python
import RPi.GPIO as GPIO  
import time  

# Configuración del pin GPIO  
ESC_PIN = 21  
GPIO.setmode(GPIO.BCM)  
GPIO.setup(ESC_PIN, GPIO.OUT)  

# Configuración de PWM  
pwm = GPIO.PWM(ESC_PIN, 50)  # 50 Hz  
pwm.start(0)  # Inicializa el PWM con un ciclo de trabajo de 0%  

# Calibración del ESC  
print("Calibrando ESC...")  
pwm.ChangeDutyCycle(5.8)  # Máxima señal  
time.sleep(20)  # Espera 2 segundos  
pwm.ChangeDutyCycle(0)  # Mínima señal  
print("Calibración completa.")   
# Limpieza  
pwm.stop()  
GPIO.cleanup()  