# -- coding: utf-8 --
#!/usr/bin/env python3
import time
import pigpio
import RPi.GPIO as GPIO
import math
import serial
from pytictoc import TicToc
import pynmea2
import shapefile
from shapely.geometry import shape, Point

# Inicialización de Pigpio
pi = pigpio.pi()
pi_m = math.pi

if not pi.connected:
    print("No se pudo conectar a pigpio. Verifica que el demonio esté corriendo.")
    exit()

# Configuración del puerto serie para GPS
port = "/dev/ttyAMA0"  
ser = serial.Serial(port, baudrate=9600, timeout=0.1)

# Configuración de pines de motor y encoder
motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23
PIN_ENCODER_A = 18
PIN_ENCODER_B = 17
PIN_ENCODER2_A = 16
PIN_ENCODER2_B = 19
INTERVALO = 0.2  # Intervalo de tiempo en segundos para cálculo de RPM

# Contadores de flancos
numero_flancos_A = 0
numero_flancos_B = 0
numero_flancos_A2 = 0
numero_flancos_B2 = 0

# Variables para RPM y RPS
RPS = 0.0
RPM = 0.0
RPS2 = 0.0
RPM2 = 0.0

# Variables control maestro esclavo
Kp_m = 0.049398
ki_m = 0.241
kp_s = 0.36612
ki_s = 1.097
Kp_m2 = 0.049398
ki_m2 = 0.241
kp_s2 = 0.36612
ki_s2 = 1.097

# Inicialización de controladores para ambos motores
rk_m, yk_m, ek_m, iek_m, upi_m = 0.0, 0.0, 0.0, 0.0, 0.0
rk_s, yk_s, ek_s, iek_s, upi_s = 0.0, 0.0, 0.0, 0.0, 0.0

rk_m2, yk_m2, ek_m2, iek_m2, upi_m2 = 0.0, 0.0, 0.0, 0.0, 0.0
rk_s2, yk_s2, ek_s2, iek_s2, upi_s2 = 0.0, 0.0, 0.0, 0.0, 0.0

# Configuración de los pines A y B con pull-up y detección de flancos
def rotary_interrupt(channel):
    global numero_flancos_A, numero_flancos_B, numero_flancos_A2, numero_flancos_B2
    if channel == PIN_ENCODER_A:
        numero_flancos_A += 1
    elif channel == PIN_ENCODER_B:
        numero_flancos_B += 1
    if channel == PIN_ENCODER2_A:
        numero_flancos_A2 += 1
    elif channel == PIN_ENCODER2_B:
        numero_flancos_B2 += 1

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_ENCODER2_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_ENCODER2_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(PIN_ENCODER_A, GPIO.BOTH, callback=rotary_interrupt)
GPIO.add_event_detect(PIN_ENCODER_B, GPIO.BOTH, callback=rotary_interrupt)
GPIO.add_event_detect(PIN_ENCODER2_A, GPIO.BOTH, callback=rotary_interrupt)
GPIO.add_event_detect(PIN_ENCODER2_B, GPIO.BOTH, callback=rotary_interrupt)

# Función para controlar el motor
def control_motor(pin_pwm, pin_dir, speed_percent, direction):
    duty_cycle = max(0, min(int(speed_percent * 255 / 100), 255))  # Limitar el ciclo de trabajo
    pi.set_PWM_dutycycle(pin_pwm, duty_cycle)
    pi.write(pin_dir, 1 if direction == 'forward' else 0)

# Función para verificar si el punto está dentro de una zona
def check(lon, lat, polygon):
    point = Point(lon, lat)
    return polygon.contains(point)

# Bucle principal de control
start_time = time.time()
setpoint_f, setpoint_W = 21.3465, 28

with open('/home/santiago/Documents/dispensador/dispen/uwurrr.txt', 'w') as output_file:
    output_file.write("Tiempo \t PWM_1 \t PWM_M2 \t W1 \t W2 \tFlujo \tFlujo2\n")
    while True:
        # Leer datos del GPS
        newdata = ser.readline().decode('utf-8').strip()
        if newdata[0:6] == "$GPRMC":
            newmsg = pynmea2.parse(newdata)
            if newmsg.status == "A":  # Señal válida
                lat, lon = newmsg.latitude, newmsg.longitude
                speed_mps = newmsg.spd_over_grnd * 0.514444  # Convertir a m/s
                # Comprobar si está en una zona definida
                poly_file = 'poligono_casona.shp'
                r = shapefile.Reader(poly_file)
                for j, shape_rec in enumerate(r.shapes()):
                    polygon = shape(shape_rec)
                    if check(lon, lat, polygon):
                        rk_m, rk_m2 = (8, 12) if j == 0 else (6, 11)
                        break
                else:
                    rk_m, rk_m2 = 0.0, 0.0
                    control_motor(motor1_pwm_pin, motor1_dir_pin, 0, 'forward')
                    control_motor(motor2_pwm_pin, motor2_dir_pin, 0, 'forward')
                    continue

        # Cálculo de la velocidad en radianes por segundo (W y W2)
        FPS1 = (numero_flancos_A + numero_flancos_B) / 1200.0
        W1 = FPS1 * (2 * pi_m / INTERVALO)

        FPS2 = (numero_flancos_A2 + numero_flancos_B2) / 1200.0
        W2 = FPS2 * (2 * pi_m / INTERVALO)

        # Control maestro-esclavo para motor 1
        ek_m = rk_m - W1
        iek_m += ek_m
        upi_m = Kp_m * ek_m + ki_m * iek_m
        upi_m = max(0, min(upi_m, 100))  # Limitar a 0-100%

        ek_s = upi_m - W1
        iek_s += ek_s
        upi_s = kp_s * ek_s + ki_s * iek_s
        upi_s = max(0, min(upi_s, 100))  # Limitar a 0-100%
        control_motor(motor1_pwm_pin, motor1_dir_pin, upi_s, 'forward')

        # Control maestro-esclavo para motor 2
        ek_m2 = rk_m2 - W2
        iek_m2 += ek_m2
        upi_m2 = Kp_m2 * ek_m2 + ki_m2 * iek_m2
        upi_m2 = max(0, min(upi_m2, 100))  # Limitar a 0-100%

        ek_s2 = upi_m2 - W2
        iek_s2 += ek_s2
        upi_s2 = kp_s2 * ek_s2 + ki_s2 * iek_s2
        upi_s2 = max(0, min(upi_s2, 100))  # Limitar a 0-100%
        control_motor(motor2_pwm_pin, motor2_dir_pin, upi_s2, 'forward')

        # Registro de datos
        ts = time.time() - start_time
        output_file.write(f"{ts:.2f}\t{upi_s:.2f}\t{upi_s2:.2f}\t{W1:.2f}\t{W2:.2f}\t{setpoint_f}\t{setpoint_f:.2f}\n")

        # Restablecer contadores de flancos
        numero_flancos_A = numero_flancos_B = numero_flancos_A2 = numero_flancos_B2 = 0

        # Esperar hasta el siguiente ciclo
        time.sleep(INTERVALO)

# Finalizar el control de motores
pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
pi.set_PWM_dutycycle(motor2_pwm_pin, 0)
pi.write(motor1_en_pin, 0)
pi.write(motor2_en_pin, 0)
pi.stop()
print('Control de motores completado.')
