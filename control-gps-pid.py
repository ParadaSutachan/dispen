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
print("Iniciando pigpio...")
pi = pigpio.pi()

if not pi.connected:
    print("No se pudo conectar a pigpio. Verifica que el demonio esté corriendo.")
    exit()

print("Conectado a pigpio.")

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

# Variables de control maestro-esclavo y de flujo
setpoint_f, setpoint_W = 21.3465, 28
delta_fn, delta_fn_1, delta_fn_2 = 0.0, 0.0, 0.0
delta_fn2, delta_fn_12, delta_fn_22 = 0.0, 0.0, 0.0
W, W2 = 0.0, 0.0
fm_n, fm_n2 = 0.0, 0.0
rk_m, rk_m2 = 0.0, 0.0
upi_m, upi_m2 = 0.0, 0.0
ek_m, ek_m2 = 0.0, 0.0
iek_m, iek_m_1, iek_m2, iek_m_12 = 0.0, 0.0, 0.0, 0.0

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

print("Configurando pines GPIO...")
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

# Función de softsensor para flujo
def softsensor(delta_W, delta_fn_1, delta_fn_2):
    return 0.1969 * delta_W + 1.359 * delta_fn_1 - 0.581 * delta_fn_2

# Bucle principal de control
start_time = time.time()

print("Iniciando bucle de control...")
with open('/home/santiago/Documents/dispensador/dispen/uwurrr.txt', 'w') as output_file:
    output_file.write("Tiempo \t PWM_1 \t PWM_M2 \t W1 \t W2 \tFlujo \tFlujo2\n")
    while True:
        try:
            # Leer datos del GPS
            newdata = ser.readline().decode('utf-8').strip()
            if newdata and newdata[0:6] == "$GPRMC":
                newmsg = pynmea2.parse(newdata)
                if newmsg.status == "A":  # Señal válida
                    lat, lon = newmsg.latitude, newmsg.longitude
                    speed_mps = newmsg.spd_over_grnd * 0.514444  # Convertir a m/s
                    print(f"GPS: Lat {lat}, Lon {lon}, Velocidad {speed_mps} m/s")

                    # Comprobar si está en una zona definida
                    poly_file = 'poligono_casona.shp'
                    r = shapefile.Reader(poly_file)
                    for j, shape_rec in enumerate(r.shapes()):
                        polygon = shape(shape_rec)
                        if check(lon, lat, polygon):
                            rk_m, rk_m2 = (8, 12) if j == 0 else (6, 11)
                            print(f"Dentro de zona {j+1}")
                            break
                    else:
                        rk_m, rk_m2 = 0.0, 0.0
                        control_motor(motor1_pwm_pin, motor1_dir_pin, 0, 'forward')
                        control_motor(motor2_pwm_pin, motor2_dir_pin, 0, 'forward')
                        print("Fuera de zona")
                        continue

            # Cálculo de la velocidad en radianes por segundo (W y W2)
            FPS1 = (numero_flancos_A + numero_flancos_B) / 1200.0
            W1 = FPS1 * (2 * math.pi / INTERVALO)

            FPS2 = (numero_flancos_A2 + numero_flancos_B2) / 1200.0
            W2 = FPS2 * (2 * math.pi / INTERVALO)

            print(f"Velocidades: Motor 1: {W1} rad/s, Motor 2: {W2} rad/s")

            # Cálculo del flujo usando softsensor para M1
            delta_W = W1 - setpoint_W
            fm_n = softsensor(delta_W, delta_fn_1, delta_fn_2)

            # Control maestro-esclavo para motor 1
            ek_m = rk_m - fm_n
            iek_m = ek_m + iek_m_1
            upi_m = 0.049398 * ek_m + 0.241 * iek_m
            upi_m = max(0, min(upi_m, 100))  # Limitar a 0-100%
            print(f"upi_m motor 1 = {upi_m}")

            # Cálculo del flujo usando softsensor para M2
            delta_W2 = W2 - setpoint_W
            fm_n2 = softsensor(delta_W2, delta_fn_12, delta_fn_22)

            # Control maestro-esclavo para motor 2
            ek_m2 = rk_m2 - fm_n2
            iek_m2 = ek_m2 + iek_m_12
            upi_m2 = 0.049398 * ek_m2 + 0.241 * iek_m2
            upi_m2 = max(0, min(upi_m2, 100))  # Limitar a 0-100%
            print(f"upi_m motor 2 = {upi_m2}")

            # Actualizar PWM de los motores
            control_motor(motor1_pwm_pin, motor1_dir_pin, upi_m, 'forward')
            control_motor(motor2_pwm_pin, motor2_dir_pin, upi_m2, 'forward')

            # Registro de datos
            ts = time.time() - start_time
            output_file.write(f"{ts:.2f}\t{upi_m:.2f}\t{upi_m2:.2f}\t{W1:.2f}\t{W2:.2f}\t{fm_n:.2f}\t{fm_n2:.2f}\n")

            # Restablecer contadores de flancos
            numero_flancos_A = numero_flancos_B = numero_flancos_A2 = numero_flancos_B2 = 0
            delta_fn_2 = delta_fn_1
            delta_fn_1 = delta_fn
            delta_fn_22 = delta_fn_12
            delta_fn_12 = delta_fn2

            # Esperar hasta el siguiente ciclo
            time.sleep(INTERVALO)

        except Exception as e:
            print(f"Error en la lectura o procesamiento: {str(e)}")
            continue

# Finalizar el control de motores
pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
pi.set_PWM_dutycycle(motor2_pwm_pin, 0)
pi.write(motor1_en_pin, 0)
pi.write(motor2_en_pin, 0)
pi.stop()
print('Control de motores completado.')
