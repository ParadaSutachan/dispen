import pigpio
import time

# Configuración de pines
PIN_ESC = 21  # Cambia esto al pin GPIO que estés utilizando para la señal del ESC

# Configuración de PWM
PWM_FREQUENCY = 50  # Frecuencia típica de ESC para motores brushless (Hz)
PWM_DUTY_CYCLE_MAX = 255  # Valor PWM máximo permitido por pigpio

# Configura la instancia de pigpio
pi = pigpio.pi()

# Verifica si se ha conectado correctamente
if not pi.connected:
    exit()

# Configura el pin GPIO como salida PWM
pi.set_mode(PIN_ESC, pigpio.OUTPUT)

# Configura el rango y frecuencia PWM
pi.set_PWM_range(PIN_ESC, PWM_DUTY_CYCLE_MAX)
pi.set_PWM_frequency(PIN_ESC, PWM_FREQUENCY)

# Mueve el motor brushless a su máxima velocidad
pi.set_PWM_dutycycle(PIN_ESC, PWM_DUTY_CYCLE_MAX)

# Espera un poco para permitir que el motor se estabilice
time.sleep(5)

# Apaga el motor
pi.set_PWM_dutycycle(PIN_ESC, 0)

# Limpieza
pi.stop()
