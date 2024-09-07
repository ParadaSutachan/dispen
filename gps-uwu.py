import RPi.GPIO as GPIO
import time

# Configuración de pines
ESC_PIN = 20  # Pin GPIO donde está conectado el ESC

# Configurar GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PIN, GPIO.OUT)

# Configurar la señal PWM para el ESC (50 Hz, típico de servos y ESC)
pwm = GPIO.PWM(ESC_PIN, 50)  # Frecuencia de 50 Hz (20ms ciclo completo)

# Inicializar ESC
pwm.start(0)  # Inicialmente con un duty cycle de 0%

# Calcular el ciclo de trabajo en función del pulso (1000-2000 microsegundos)
def set_speed(pulse_width):
    duty_cycle = pulse_width / 20000 * 100  # Convertir microsegundos a ciclo de trabajo
    pwm.ChangeDutyCycle(duty_cycle)

try:
    # Configurar ESC a velocidad máxima (2000 microsegundos)
    print("Acelerando a velocidad máxima...")
    set_speed(2000)
    
    time.sleep(5)  # Mantener la velocidad por 5 segundos
    
finally:
    # Detener el motor y limpiar la configuración
    set_speed(0)
    pwm.stop()
    GPIO.cleanup()

