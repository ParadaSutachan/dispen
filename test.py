import RPi.GPIO as GPIO  
import time  
import pigpio
import math

# Inicialización de Pigpio
pi = pigpio.pi()
pi_m = math.pi

# Pines de motor y encoder
motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23

# Variables para flancos
FLANCOS_M1_A = 0
FLANCOS_M1_B = 0
FLANCOS_M1 = 0  # Suma total de los flancos en A y B

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
W = 0

# Función de interrupción para manejar los cambios en los pines A y B
def rotary_interrupt(channel):
    global count
    global FLANCOS_M1_A, FLANCOS_M1_B, FLANCOS_M1
    
    if channel == pin_a:
        FLANCOS_M1_A += 1  # Incrementa el contador de flancos en pin A
    elif channel == pin_b:
        FLANCOS_M1_B += 1  # Incrementa el contador de flancos en pin B
    
    # Sumar los flancos detectados en A y B
    FLANCOS_M1 = FLANCOS_M1_A + FLANCOS_M1_B

    state_a = GPIO.input(pin_a)
    state_b = GPIO.input(pin_b)

    # Detectar el cambio en ambos pines A y B para incrementar o decrementar el contador
    if state_a != state_b:
        count += 1  # Incremento
    else:
        count -= 1  # Decremento

# Configuración GPIO
GPIO.setmode(GPIO.BCM)

# Configuración de los pines A y B con pull-up y detección de ambos flancos
GPIO.setup(pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Añadir detección de eventos en ambos pines para detectar flancos ascendentes y descendentes
GPIO.add_event_detect(pin_a, GPIO.BOTH, callback=rotary_interrupt)
GPIO.add_event_detect(pin_b, GPIO.BOTH, callback=rotary_interrupt)

try:
    while True:
        # Habilitar motores
        pi.write(motor1_en_pin, 1)
        pi.write(motor2_en_pin, 1)

        # Controlar motor 1
        control_motor(motor1_pwm_pin, motor1_dir_pin, 100, 'forward')

        # Imprimir el conteo del encoder
        print("Count: {}".format(count))

        # Imprimir los flancos totales detectados en A y B
        print("Flancos en A: {}, Flancos en B: {}, Flancos Totales (M1): {}".format(FLANCOS_M1_A, FLANCOS_M1_B, FLANCOS_M1))

        # Calcular FPS y W
        FPS = FLANCOS_M1 / 1200.0
        W = FPS*((2 * pi_m) / 0.2)
        print("W: {}".format(W))

        # Reiniciar el contador
        count = 0  
        FLANCOS_M1_A=0.0
        FLANCOS_M1_B=0.0  
        FLANCOS_M1=0.0
        time.sleep(0.2)

except KeyboardInterrupt:
    GPIO.cleanup()
