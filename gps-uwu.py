import pigpio
import time

# Configuración
ESC_PIN = 19  # El pin GPIO donde está conectado el ESC
pi = pigpio.pi()

if not pi.connected:
    exit()

# Inicialización del ESC
ESC_MIN = 1000  # Microsegundos para velocidad mínima (parado)
ESC_MAX = 2000  # Microsegundos para máxima velocidad

# Calibrar ESC
print("Calibrando ESC...")
pi.set_servo_pulsewidth(ESC_PIN, 0)  # Detener motor
time.sleep(2)

pi.set_servo_pulsewidth(ESC_PIN, ESC_MAX)  # Señal máxima
print("Enviando señal máxima para calibración.")
time.sleep(2)

pi.set_servo_pulsewidth(ESC_PIN, ESC_MIN)  # Señal mínima
print("Enviando señal mínima para calibración.")
time.sleep(2)

# Aumentar a máxima velocidad
print("Subiendo a máxima velocidad.")
pi.set_servo_pulsewidth(ESC_PIN, ESC_MAX)
time.sleep(5)  # Mantener a máxima velocidad por 5 segundos

# Detener motor
print("Deteniendo motor.")
pi.set_servo_pulsewidth(ESC_PIN, ESC_MIN)
time.sleep(2)

# Limpiar y desconectar
pi.set_servo_pulsewidth(ESC_PIN, 0)
pi.stop()
print("Proceso finalizado.")

