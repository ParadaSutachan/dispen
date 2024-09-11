import time
import pigpio
import RPi.GPIO as GPIO
import math
import serial
import pynmea2
import shapefile
from shapely.geometry import shape, Point
import asyncio  # Asincronía para GPS y lectura de datos

# Configuración inicial
pi = pigpio.pi()
port = "/dev/ttyAMA0"
ser = serial.Serial(port, baudrate=9600, timeout=0.1)

# Pines de motores y encoders
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

# Pines para los ESC
ESC_PIN_1 = 21
ESC_PIN_2 = 20

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

# Variables de flujo
fm_n= 0.0
fm_n2= 0.0
W=0.0
W2= 0.0

# Variables control maestro esclavo

Kp_m = 0.049398
ki_m = 0.241
kp_s = 0.36612
ki_s = 1.097

Kp_m2 = 0.049398
ki_m2 = 0.241
kp_s2 = 0.36612
ki_s2 = 1.097

rk_m= 0.0
yk_m= 0.0
ek_m= 0.0
iek_m= 0.0
iek_m_1= 0.0    #PARA M1
upi_m= 0.0
up_m = 0.0
ui_m = 0.0

rk_m2= 0.0
yk_m2= 0.0
ek_m2= 0.0
iek_m2= 0.0     #PARA M2
iek_m_12= 0.0
upi_m2= 0.0
up_m2 = 0.0
ui_m2 = 0.0

rk_s= 0.0
yk_s= 0.0
ek_s= 0.0
iek_s= 0.0
iek_s_1= 0.0    #PARA M1
upi_s= 0.0
up_s = 0.0
ui_s = 0.0

rk_s2= 0.0
yk_s2= 0.0
ek_s2= 0.0
iek_s2= 0.0        #PARA M2
iek_s_12= 0.0   
upi_s2= 0.0
up_s2 = 0.0
ui_s2 = 0.0
k= 0.0
k2=0.0

setpoint_f= 21.3465
setpoint_W = 28
delta_fn= 0.0
delta_fn_2= 0.0

# Inicializar GPIO y PWM para ESC
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PIN_1, GPIO.OUT)
GPIO.setup(ESC_PIN_2, GPIO.OUT)
pwm1 = GPIO.PWM(ESC_PIN_1, 50)
pwm2 = GPIO.PWM(ESC_PIN_2, 50)
pwm1.start(0)
pwm2.start(0)

# Constantes del sistema
ancho_faja = 3.6
rate = 12.82

# Función para convertir microsegundos a duty cycle para ESC
def set_speed(pwm, pulse_width):
    duty_cycle = pulse_width / 20000 * 100
    pwm.ChangeDutyCycle(duty_cycle)

# Funciones de callback para contar flancos
def contador_flancos_encoder(gpio, level, tick):
    global numero_flancos_A
    numero_flancos_A += 1

def contador_flancos_encoder_b(gpio, level, tick):
    global numero_flancos_B
    numero_flancos_B += 1

def contador_flancos_encoder2(gpio, level, tick):
    global numero_flancos_A2
    numero_flancos_A2 += 1

def contador_flancos_encoder_b2(gpio, level, tick):
    global numero_flancos_B2
    numero_flancos_B2 += 1

# Configuración de pines de entrada para los encoders
pi.set_mode(PIN_ENCODER_A, pigpio.INPUT)
pi.set_pull_up_down(PIN_ENCODER_A, pigpio.PUD_UP)
pi.set_mode(PIN_ENCODER_B, pigpio.INPUT)
pi.set_pull_up_down(PIN_ENCODER_B, pigpio.PUD_UP)

pi.set_mode(PIN_ENCODER2_A, pigpio.INPUT)
pi.set_pull_up_down(PIN_ENCODER2_A, pigpio.PUD_UP)
pi.set_mode(PIN_ENCODER2_B, pigpio.INPUT)
pi.set_pull_up_down(PIN_ENCODER2_B, pigpio.PUD_UP)

# Configuración de callbacks
cb1 = pi.callback(PIN_ENCODER_A, pigpio.EITHER_EDGE, contador_flancos_encoder)
cb3 = pi.callback(PIN_ENCODER2_A, pigpio.EITHER_EDGE, contador_flancos_encoder2)

# Función asincrónica para leer el GPS
async def leer_gps():
    while True:
        newdata = ser.readline().decode('utf-8').strip()
        if newdata.startswith("$GPRMC"):
            newmsg = pynmea2.parse(newdata)
            if newmsg.status == "A":
                lat = newmsg.latitude
                lon = newmsg.longitude
                speed = newmsg.spd_over_grnd * 0.514444  # Convertimos a m/s
                return lat, lon, speed
        await asyncio.sleep(0.1)  # Evitar bloqueo del sistema

# Función para verificar si estamos en la zona
def check_zone(lon, lat, shapes):
    point = Point(lon, lat)
    for j, polygon in enumerate(shapes):
        if polygon.contains(point):
            return j + 1
    return None

# Cargar shapefile de zonas
def cargar_shapefile(archivo):
    r = shapefile.Reader(archivo)
    shapes = [shape(s) for s in r.shapes()]
    return shapes

# Cálculo de la dosificación en función de la zona
def calcular_dosificacion(zona, rate):
    if zona == 1:
        return 0.7 * rate, 0.3 * rate
    elif zona == 2:
        return 0.4 * rate, 0.6 * rate
    else:
        return 0.0, 0.0
    
def control_motor(pin_pwm, pin_dir, speed_percent, direction):
    duty_cycle = int(speed_percent * 255 / 100)
    pi.set_PWM_dutycycle(pin_pwm, duty_cycle)

    if direction == 'forward':
        pi.write(pin_dir, 1)  # Dirección hacia adelante
    elif direction == 'backward':
        pi.write(pin_dir, 0)  # Dirección hacia atrás
    else:
        raise ValueError("Dirección no válida. Usa 'forward' o 'backward'.")

# Función principal
async def main():
    shapes = cargar_shapefile('poligono_casona.shp')
    output_file_path = '/home/santiago/Documents/dispensador/dispen/uwurrr.txt'
    setpoint_W = 28
    setpoint_f = 21.3465

    with open(output_file_path, 'w') as output_file:
        output_file.write("Tiempo\tPWM_1\tPWM_2\tVel_1\tVel_2\tFlujo_1\tFlujo_2\n")
        while True:
            lat, lon, speed_mps = await leer_gps()
            zona = check_zone(lon, lat, shapes)
            if zona:
                dosis_m1, dosis_m2 = calcular_dosificacion(zona, rate)
            else:
                dosis_m1, dosis_m2 = 0.0, 0.0
                print("Fuera de zona")
            
            print("Moviendo los brushless a velocidad de 1200 microsegundos...")
            set_speed(pwm1, 1200)  # Señal de 1200 microsegundos para el primer motor
            set_speed(pwm2, 1200)  # Señal de 1200 microsegundos para el segundo moto

            # Calcular referencias de control (lógica dejada intacta)
            rk_m = float(speed_mps * ancho_faja * dosis_m1)
            rk_m2 = float(speed_mps * ancho_faja * dosis_m2)

            # Restablecer todas las variables tal como estaban
            # (Mantenido todo el bloque original de maestro-esclavo)

            # Calcular la velocidad del motor a partir de los flancos
            flancos_totales_1 = numero_flancos_A + numero_flancos_B
            RPS = flancos_totales_1 / (600.0)  # Usando el tiempo de INTERVALO
            W = RPS * ((2 * math.pi) / INTERVALO)

            flancos_totales_2 = numero_flancos_A2 + numero_flancos_B2
            RPS2 = flancos_totales_2 / (600.0)
            W2 = RPS2 * ((2 * math.pi) / INTERVALO)

            # Control maestro para M1
            yk_m = fm_n
            ek_m = rk_m - yk_m
            iek_m = ek_m + iek_m_1
            up_m = Kp_m * ek_m
            ui_m = ki_m * iek_m
            upi_m = up_m + ui_m

            if upi_m < 0 or upi_m > 100:
                if upi_m < 0:
                    ui_m = 0 - up_m
                if upi_m > 100:
                    ui_m = 100 - up_m

            upi_m = ui_m + up_m
            print("upi_m = " + str(upi_m))

            # Control maestro para M2
            yk_m2 = fm_n2
            ek_m2 = rk_m2 - yk_m2
            iek_m2 = ek_m2 + iek_m_12
            up_m2 = Kp_m2 * ek_m2
            ui_m2 = ki_m2 * iek_m2
            upi_m2 = up_m2 + ui_m2

            if upi_m2 < 0 or upi_m2 > 100:
                if upi_m2 < 0:
                    ui_m2 = 0 - up_m2
                if upi_m2 > 100:
                    ui_m2 = 100 - up_m2

            upi_m2 = ui_m2 + up_m2
            print("upi_m motor 2 = " + str(upi_m2))

            # Control esclavo para M1
            rk_s = upi_m
            yk_s = W
            ek_s = rk_s - yk_s
            iek_s = ek_s + iek_s_1
            ui_s = ki_s * iek_s
            up_s = kp_s * ek_s
            upi_s = up_s + ui_s

            if upi_s < 0 or upi_s > 100:
                if upi_s < 0:
                    ui_s = 0 - up_s
                if upi_s > 100:
                    ui_s = 100 - up_s

            upi_s = ui_s + up_s
            print("pwm motor 1 = " + str(upi_s))

            # Control esclavo para M2
            rk_s2 = upi_m2
            yk_s2 = W2
            ek_s2 = rk_s2 - yk_s2
            iek_s2 = ek_s2 + iek_s_12
            ui_s2 = ki_s2 * iek_s2
            up_s2 = kp_s2 * ek_s2
            upi_s2 = up_s2 + ui_s2

            if upi_s2 < 0 or upi_s2 > 100:
                if upi_s2 < 0:
                    ui_s2 = 0 - up_s2
                if upi_s2 > 100:
                    ui_s2 = 100 - up_s2

            upi_s2 = ui_s2 + up_s2
            print("pwm motor 2 = " + str(upi_s2))

            # Aplicar los valores de control a los motores
            control_motor(motor1_pwm_pin, motor1_dir_pin, upi_s, 'forward')
            control_motor(motor2_pwm_pin, motor2_dir_pin, upi_s2, 'forward')

            # Guardar los datos en el archivo
            ts = time.time()
            output_file.write(f"{ts:.2f}\t{upi_s:.2f}\t{upi_s2:.2f}\t{W:.2f}\t{W2:.2f}\t{fm_n:.2f}\t{fm_n2:.2f}\n")

            await asyncio.sleep(0.2)

# Iniciar el programa
asyncio.run(main())

# Deshabilitar motores
pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
pi.set_PWM_dutycycle(motor2_pwm_pin, 0)
pi.write(motor1_en_pin, 0)
pi.write(motor2_en_pin, 0)

# Detener los motores
print("Deteniendo motores...")
set_speed(pwm1, 1000)  # Señal mínima para detener el primer motor
set_speed(pwm2, 1000)  # Señal mínima para detener el segundo motor
# Limpiar y detener PWM
pwm1.stop()
pwm2.stop()
GPIO.cleanup()
print("Proceso finalizado.")
# Detener Pigpio
pi.stop()
print('Tiempo de funcionamiento de los motores completado.')
