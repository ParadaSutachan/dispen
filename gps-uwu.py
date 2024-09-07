import RPi.GPIO as GPIO
import time

# Configuración de pines
ESC_PIN_1 = 19  # Pin GPIO donde está conectado el primer ESC
ESC_PIN_2 = 20  # Pin GPIO donde está conectado el segundo ESC

# Configurar GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PIN_1, GPIO.OUT)
GPIO.setup(ESC_PIN_2, GPIO.OUT)

# Configurar la señal PWM para los ESC (50 Hz)
pwm1 = GPIO.PWM(ESC_PIN_1, 50)  # Frecuencia de 50 Hz (20ms ciclo completo)
pwm2 = GPIO.PWM(ESC_PIN_2, 50)  # Frecuencia de 50 Hz (20ms ciclo completo)

# Inicializar PWM
pwm1.start(0)  # Iniciar con un duty cycle de 0%
pwm2.start(0)  # Iniciar con un duty cycle de 0%

# Función para convertir microsegundos a ciclo de trabajo
def set_speed(pwm, pulse_width):
    duty_cycle = pulse_width / 20000 * 100  # Convertir microsegundos a ciclo de trabajo
    pwm.ChangeDutyCycle(duty_cycle)

try:
    # Mover los motores a velocidad de 1200 microsegundos por 5 segundos
    print("Moviendo los motores a velocidad de 1200 microsegundos...")
    set_speed(pwm1, 1200)  # Señal de 1200 microsegundos para el primer motor
    set_speed(pwm2, 1200)  # Señal de 1200 microsegundos para el segundo motor
    time.sleep(5)  # Mantener la velocidad por 5 segundos

    # Detener los motores
    print("Deteniendo motores...")
    set_speed(pwm1, 1000)  # Señal mínima para detener el primer motor
    set_speed(pwm2, 1000)  # Señal mínima para detener el segundo motor
    time.sleep(1)
    
finally:
    # Limpiar y detener PWM
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
    print("Proceso finalizado.")

