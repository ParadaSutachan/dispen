import pigpio
import time

# Configuración de pines
PIN_ESC = 21  # Cambia esto al pin GPIO que estés utilizando para la señal del ESC

# Configura la instancia de pigpio
pi = pigpio.pi()

# Verifica si se ha conectado correctamente
if not pi.connected:
    exit()

# Configuración de la señal PWM
PWM_FREQUENCY = 50  # Frecuencia típica de ESC para motores brushless (Hz)
PWM_DUTY_CYCLE_MAX = 2000  # Valor PWM máximo para el ESC (esto puede variar)

# Configura el pin GPIO como salida PWM
pi.set_mode(PIN_ESC, pigpio.OUTPUT)

# Mueve el motor brushless a su máxima velocidad
pi.set_PWM_frequency(PIN_ESC, PWM_FREQUENCY)
pi.set_PWM_range(PIN_ESC, 1000)  # Rango del PWM (ajusta si es necesario)
pi.set_PWM_dutycycle(PIN_ESC, PWM_DUTY_CYCLE_MAX)

# Espera un poco para permitir que el motor se estabilice
time.sleep(5)

# Apaga el motor
pi.set_PWM_dutycycle(PIN_ESC, 0)

# Limpieza
pi.stop()
