import RPi.GPIO as GPIO
import time

# Configuración del pin GPIO
ESC_PIN = 18  # Puedes usar otro pin GPIO si lo prefieres

# Configuración de la señal PWM
PWM_FREQUENCY = 50  # Frecuencia en Hz para el ESC
MIN_PULSE_WIDTH = 1000  # Ancho de pulso mínimo en microsegundos
MAX_PULSE_WIDTH = 2000  # Ancho de pulso máximo en microsegundos

# Configuración de GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PIN, GPIO.OUT)

# Configuración de PWM
pwm = GPIO.PWM(ESC_PIN, PWM_FREQUENCY)
pwm.start(0)  # Comienza con un ciclo de trabajo del 0%

def set_pwm(pulse_width):
    """Configura el ancho del pulso PWM."""
    duty_cycle = (pulse_width / 1000000.0) * PWM_FREQUENCY * 100.0  # Calcula el ciclo de trabajo en porcentaje
    pwm.ChangeDutyCycle(duty_cycle)

try:
    print("Calibración del ESC: Sigue las instrucciones del manual del ESC.")

    # 1. Configura el ESC a su valor mínimo
    print("Configurando al valor mínimo...")
    set_pwm(MIN_PULSE_WIDTH)
    time.sleep(2)  # Espera para asegurar que el ESC detecte el valor

    # 2. Configura el ESC a su valor máximo
    print("Configurando al valor máximo...")
    set_pwm(MAX_PULSE_WIDTH)
    time.sleep(2)  # Espera para asegurar que el ESC detecte el valor

    # 3. Configura el ESC al valor medio
    print("Configurando al valor medio...")
    set_pwm((MIN_PULSE_WIDTH + MAX_PULSE_WIDTH) / 2)
    time.sleep(2)  # Espera para asegurar que el ESC detecte el valor

    # Instrucciones adicionales si el ESC requiere confirmación
    print("Si el ESC requiere confirmación, sigue las instrucciones del manual.")
    print("Proceso de calibración completo.")

finally:
    pwm.stop()
    GPIO.cleanup()
