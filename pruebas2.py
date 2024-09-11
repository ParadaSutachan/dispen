import RPi.GPIO as GPIO  
import time  
import pigpio

# Inicialización de Pigpio
pi = pigpio.pi()

# Pines de motor y encoder
motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23

# Función para controlar el motor
def control_motor(pin_pwm, pin_dir, speed_percent, direction):
    duty_cycle = int(speed_percent * 255 / 100)
    pi.set_PWM_dutycycle(pin_pwm, duty_cycle)

    if direction == 'forward':
        pi.write(pin_dir, 1)  # Dirección hacia adelante
    elif direction == 'backward':
        pi.write(pin_dir, 0)  # Dirección hacia atrás
    else:
        raise ValueError("Dirección no válida. Usa 'forward' o 'backward'.")
# Configuración de pines  
pin_a = 17  # Pin A del encoder  
pin_b = 18  # Pin B del encoder  

# Variables  
count = 0  
last_state = 0  

def rotary_interrupt(channel):  
    global count  
    global last_state  

    if GPIO.input(pin_a) != last_state:  
        if GPIO.input(pin_b) != last_state:  
            count += 1  
        else:  
            count -= 1  
    last_state = GPIO.input(pin_a)  

# Configuración GPIO  
GPIO.setmode(GPIO.BCM)  
GPIO.setup(pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.setup(pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.add_event_detect(pin_a, GPIO.BOTH, callback=rotary_interrupt)  

try:  
    while True:  
        pi.write(motor1_en_pin, 1)
        pi.write(motor2_en_pin, 1)

        print("Count: {}".format(count))
        count = 0    
        time.sleep(0.2)  
except KeyboardInterrupt:  
    GPIO.cleanup() # -- coding: utf-8 --