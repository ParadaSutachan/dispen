import RPi.GPIO as GPIO  
import time  

# Configuración de los pines  
PWM_A_PIN = 18  # Pin para PWM Motor A  
DIR_A_PIN = 17  # Pin para Dirección Motor A  

# Configuración de GPIO  
GPIO.setmode(GPIO.BCM)  
GPIO.setup(PWM_A_PIN, GPIO.OUT)  
GPIO.setup(DIR_A_PIN, GPIO.OUT)  

# Configuración del PWM  
pwm_a = GPIO.PWM(PWM_A_PIN, 1000)  # Frecuencia de 1 kHz  
pwm_a.start(0)  # Iniciar el PWM con ciclo de trabajo de 0%  

# Función para mover el motor  
def move_motor_a(speed, direction):  
    GPIO.output(DIR_A_PIN, direction)  # Cambia la dirección  
    pwm_a.ChangeDutyCycle(speed)  # Cambia la velocidad del motor  

try:  
    print("Starting Motor A in Forward Direction")  
    move_motor_a(100, GPIO.HIGH)  # 100% velocidad en dirección alta  
    time.sleep(5)  # Mantener el motor encendido durante 5 segundos  

    print("Stopping Motor A")  
    move_motor_a(0, GPIO.HIGH)  # Detener el motor  

finally:  
    pwm_a.stop()  # Detener el PWM  
    GPIO.cleanup()  # Limpiar la configuración de GPIO