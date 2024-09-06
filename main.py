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
pwm.ChangeDutyCycle(100)  # Máxima señal  
time.sleep(2)  # Espera 2 segundos  
pwm.ChangeDutyCycle(0)  # Mínima señal  
time.sleep(2)  # Espera 2 segundos  
print("Calibración completa.")  

# Control del motor  
try:  
    while True:  
        for duty_cycle in range(0, 101, 5):  # Aumenta el ciclo de trabajo  
            pwm.ChangeDutyCycle(duty_cycle)  
            time.sleep(0.5)  
        for duty_cycle in range(100, -1, -5):  # Disminuye el ciclo de trabajo  
            pwm.ChangeDutyCycle(duty_cycle)  
            time.sleep(0.5)  
except KeyboardInterrupt:  
    pass  

# Limpieza  
pwm.stop()  
GPIO.cleanup()