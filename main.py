# -- coding: utf-8 --
import time
import pigpio

# Pines de los brushless
PWM_PIN = 18  # Ajusta este valor según el pin que estés usando
FREQUENCY = 50  # Frecuencia típica para PWM de motores brushless es de 50 Hz

# Inicialización de Pigpio
pi = pigpio.pi()

# Configuración del PWM para el motor brushless
pi.set_PWM_frequency(PWM_PIN, FREQUENCY)

# Iniciar el PWM con 100% del ciclo de trabajo
# El ciclo de trabajo 255 es 100% en una escala de 0 a 255
pi.set_PWM_dutycycle(PWM_PIN, 255)

try:
    # El motor funcionará al 100% hasta que detengas el script
    print("Motor brushless funcionando al 100%")
    while True:
        time.sleep(1)  # Mantener el motor funcionando

except KeyboardInterrupt:
    # Al presionar Ctrl+C se detendrá el motor y se limpiará
    print("Interrupción detectada. Deteniendo el motor.")
    pi.set_PWM_dutycycle(PWM_PIN, 0)  # Detener el motor
    pi.stop()
