import pigpio  
import time  

# Configuración del pin GPIO  
ESC_PIN = 21  
pi = pigpio.pi()  # Conectar al daemon pigpio  

if not pi.connected:  
    exit()  

# Calibración del ESC  
print("Calibrando ESC...")  
pi.set_servo_pulsewidth(ESC_PIN, 2000)  # Máxima señal  
time.sleep(2)  # Espera 2 segundos  
pi.set_servo_pulsewidth(ESC_PIN, 1000)  # Mínima señal  
time.sleep(2)  # Espera 2 segundos  
print("Calibración completa.")  

# Control del motor  
try:  
    while True:  
        for pulsewidth in range(1000, 2001, 50):  # Aumenta el pulso  
            pi.set_servo_pulsewidth(ESC_PIN, pulsewidth)  
            time.sleep(0.5)  
        for pulsewidth in range(2000, 999, -50):  # Disminuye el pulso  
            pi.set_servo_pulsewidth(ESC_PIN, pulsewidth)  
            time.sleep(0.5)  
except KeyboardInterrupt:  
    pass  

# Limpieza  
pi.set_servo_pulsewidth(ESC_PIN, 0)  # Detener el motor  
pi.stop()