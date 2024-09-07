import RPi.GPIO as GPIO
import time

# Configurar los pines GPIO
ESC_PIN = 20  # El pin GPIO donde está conectado el ESC
pwmMin = 1000  # Valor mínimo del PWM en microsegundos
pwmMax = 2000  # Valor máximo del PWM en microsegundos

# Configurar GPIO en modo BCM y el pin de salida PWM
GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PIN, GPIO.OUT)

# Configurar PWM en 50Hz (frecuencia estándar para un ESC)
pwm = GPIO.PWM(ESC_PIN, 50)
pwm.start(0)

# Función para convertir microsegundos a duty cycle para PWM en 50Hz
def set_pwm_duty_cycle(pulse_width_microseconds):
    duty_cycle = pulse_width_microseconds / 20000 * 100  # Convertir microsegundos a porcentaje de duty cycle
    pwm.ChangeDutyCycle(duty_cycle)

try:
    # Inicializar el ESC en el valor mínimo durante unos segundos
    print("Inicializando ESC...")
    set_pwm_duty_cycle(pwmMin)
    time.sleep(3)

    # Subir a un valor neutro (1500 microsegundos)
    print("Subiendo al valor neutro (1500 us)...")
    set_pwm_duty_cycle(1500)
    time.sleep(2)

    while True:
        # Aumentar la señal PWM desde el mínimo hasta el máximo
        print("Acelerando...")
        for pulse_width in range(pwmMin, pwmMax + 1, 10):
            set_pwm_duty_cycle(pulse_width)
            time.sleep(0.02)  # Pequeña pausa entre incrementos

        time.sleep(2)  # Mantener la velocidad máxima por 2 segundos

        # Reducir la señal PWM desde el máximo hasta el mínimo
        print("Desacelerando...")
        for pulse_width in range(pwmMax, pwmMin - 1, -10):
            set_pwm_duty_cycle(pulse_width)
            time.sleep(0.02)  # Pequeña pausa entre decrementos

        time.sleep(2)  # Mantener la velocidad mínima por 2 segundos

except KeyboardInterrupt:
    print("Parando...")

finally:
    pwm.stop()  # Detener el PWM
    GPIO.cleanup()  # Limpiar los pines GPIO al finalizar
