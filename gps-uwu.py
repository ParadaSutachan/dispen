import RPi.GPIO as GPIO
import time

# Configuración de pines
ESC_PIN = 19  # Pin GPIO donde está conectado el ESC

# Configurar GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PIN, GPIO.OUT)

# Configurar la señal PWM para el ESC (50 Hz)
pwm = GPIO.PWM(ESC_PIN, 50)  # Frecuencia de 50 Hz (20ms ciclo completo)

# Inicializar PWM
pwm.start(0)  # Iniciar con un duty cycle de 0%

# Función para convertir microsegundos a ciclo de trabajo
def set_speed(pulse_width):
    duty_cycle = pulse_width / 20000 * 100  # Convertir microsegundos a ciclo de trabajo
    pwm.ChangeDutyCycle(duty_cycle)

try:
    # Mover el motor a máxima velocidad por 3 segundos
    print("Moviendo a máxima velocidad...")
    set_speed(1200)  # Señal máxima (2000 microsegundos)
    time.sleep(5)  # Mantener la velocidad por 3 segundos

    # Detener el motor
    print("Deteniendo motor...")
    set_speed(1000)  # Señal mínima (1000 microsegundos para detener)
    time.sleep(1)
    
finally:
    # Limpiar y detener PWM
    pwm.stop()
    GPIO.cleanup()
    print("Proceso finalizado.")

