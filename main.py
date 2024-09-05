import RPi.GPIO as GPIO
import time

# Configuración del pin GPIO
ESC_PIN = 18  # Puedes usar otro pin GPIO si lo prefieres

# Configuración de la señal PWM
PWM_FREQUENCY = 50  # Frecuencia en Hz para el ESC
MIN_PULSE_WIDTH = 1000  # Ancho de pulso mínimo en microsegundos (puede variar según el ESC)
MAX_PULSE_WIDTH = 2000  # Ancho de pulso máximo en microsegundos (puede variar según el ESC)
PULSE_WIDTH = MIN_PULSE_WIDTH  # Valor inicial del ancho de pulso

# Configuración de GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PIN, GPIO.OUT)

# Configuración de PWM
pwm = GPIO.PWM(ESC_PIN, PWM_FREQUENCY)
pwm.start(0)  # Comienza con un ciclo de trabajo del 0%

def set_speed(speed_percentage):
    """Configura la velocidad del ESC basándose en el porcentaje de velocidad."""
    global PULSE_WIDTH
    if 0 <= speed_percentage <= 100:
        PULSE_WIDTH = MIN_PULSE_WIDTH + (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) * (speed_percentage / 100.0)
        duty_cycle = (PULSE_WIDTH / 1000000.0) * PWM_FREQUENCY * 100.0  # Calcula el ciclo de trabajo en porcentaje
        pwm.ChangeDutyCycle(duty_cycle)
    else:
        print("Error: El porcentaje de velocidad debe estar entre 0 y 100.")

try:
    # Ejemplo: aumentar la velocidad de 0 a 100% en pasos
    for speed in range(0, 101, 10):
        set_speed(speed)
        print(f"Velocidad: {speed}%")
        time.sleep(2)
    
    # Ejemplo: disminuir la velocidad de 100 a 0% en pasos
    for speed in range(100, -1, -10):
        set_speed(speed)
        print(f"Velocidad: {speed}%")
        time.sleep(2)

finally:
    pwm.stop()
    GPIO.cleanup()
