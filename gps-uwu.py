import RPi.GPIO as GPIO
import time

# Configuración de pines
ESC_PIN = 19  # Pin GPIO donde está conectado el ESC

# Configurar GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PIN, GPIO.OUT)

# Configurar la señal PWM para el ESC (50 Hz)
pwm = GPIO.PWM(ESC_PIN, 50)  # Frecuencia de 50 Hz (20 ms ciclo completo)

# Inicializar PWM
pwm.start(0)  # Iniciar con un duty cycle de 0%

# Función para convertir microsegundos a ciclo de trabajo
def set_speed(pulse_width):
    duty_cycle = pulse_width / 20000 * 100  # Convertir microsegundos a ciclo de trabajo
    pwm.ChangeDutyCycle(duty_cycle)

try:
    print("Iniciando calibración ESC...")
    
    # Enviar señal de máxima potencia para calibración
    print("Enviando señal máxima...")
    set_speed(2000)  # Señal máxima (2000 microsegundos)
    time.sleep(2)
    
    # Enviar señal mínima para restablecer configuración
    print("Enviando señal mínima para calibración...")
    set_speed(1000)  # Señal mínima (1000 microsegundos)
    time.sleep(2)
    
    # Esperar y detener el motor
    print("Calibración completada, deteniendo motor...")
    set_speed(0)
    time.sleep(1)
    
finally:
    pwm.stop()
    GPIO.cleanup()
    print("Proceso de calibración terminado.")

