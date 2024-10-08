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
pi_m = math.pi

if not pi.connected:
    print("No se pudo conectar a pigpio. Verifica que el demonio esté corriendo.")
    exit()

print("Conectado a pigpio.")

# Configura el puerto serie  
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
k= 0.0

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
# Variable Voltaje
v1 = 0.0
v2 = 0.0
rate= 12.82
FLANCOS_M1= 0.0
FLANCOS_M2=0.0
FLANCOS_M1_A=0.0
FLANCOS_M1_B=0.0
FLANCOS_M1_A2=0.0
FLANCOS_M1_B2=0.0
FPS=0.0
FPS2=0.0

def rotary_interrupt(channel):

    global FLANCOS_M1_A, FLANCOS_M1_B, FLANCOS_M1,FLANCOS_M1_A2,FLANCOS_M1_B2, FLANCOS_M2
    
    if channel == PIN_ENCODER_A:
        FLANCOS_M1_A += 1  # Incrementa el contador de flancos en pin A
    elif channel == PIN_ENCODER_B:
        FLANCOS_M1_B += 1  # Incrementa el contador de flancos en pin B

    if channel == PIN_ENCODER2_A:
        FLANCOS_M1_A2 += 1  # Incrementa el contador de flancos en pin A
    elif channel == PIN_ENCODER2_B:
        FLANCOS_M1_B2 += 1  # Incrementa el contador de flancos en pin B

    
    # Sumar los flancos detectados en A y B
    FLANCOS_M1 = FLANCOS_M1_A + FLANCOS_M1_B
    FLANCOS_M2= FLANCOS_M1_A2 + FLANCOS_M1_B2

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
k=0.0
k2 = 0.0
# Bucle principal de control
start_time = time.time()

print("Iniciando bucle de control...")
with open('/home/santiago/Documents/dispensador/dispen/uwurrr.txt', 'w') as output_file:
    output_file.write("Tiempo \t PWM_1 \t PWM_M2 \t W1 \t W2 \tFlujo \tFlujo2\n")
    while True:
        try:
            k =+ 1
            k =+ 2
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
        # Calcular FPS y W
            FPS = FLANCOS_M1 / 1200.0
            W1 = FPS*((2 * pi_m) / 0.2)
            print("W: " + str(W))


            # Calcular FPS y W
            FPS2 = FLANCOS_M2 / 1200.0
            W = FPS2*((2 * pi_m) / 0.2)
            print("W2: " + str(W))

            print(f"Velocidades: Motor 1: {W1} rad/s, Motor 2: {W2} rad/s")

            # Cálculo del flujo usando softsensor para M1
            delta_W = W1 - setpoint_W
            fm_n = softsensor(delta_W, delta_fn_1, delta_fn_2)
            if k <= 3:
                fm_n= 0
            else :
                fm_n = delta_fn + setpoint_f 

            #Control maestro para M1
            yk_m = fm_n
            ek_m= rk_m - yk_m
            iek_m = ek_m + iek_m_1
            up_m = Kp_m*ek_m
            ui_m = ki_m*iek_m
            upi_m = up_m  + ui_m
            if upi_m < 0 or upi_m > 100:
                if upi_m < 0 :
                    ui_m = 0 - up_m
                if upi_m >100:
                    ui_m = 100 - up_m
            upi_m = ui_m  + up_m
            print("upi_m = "+ str(upi_m))

            # Cálculo del flujo usando softsensor para M2
            delta_W2 = W2 - setpoint_W
            fm_n2 = softsensor(delta_W2, delta_fn_12, delta_fn_22)
            if k2 <= 3:
                fm_n2= 0
            else :                                                          # Softsensor PARA M2
                fm_n2 = delta_fn2 + setpoint_f

            #Control maestro para M2
            yk_m2 = fm_n2
            ek_m2= rk_m2 - yk_m2
            iek_m2 = ek_m2 + iek_m_12
            up_m2 = Kp_m2*ek_m2
            ui_m2 = ki_m2*iek_m2
            upi_m2 = up_m2  + ui_m2
            if upi_m2 < 0 or upi_m2 > 100:
                if upi_m2 < 0 :
                    ui_m2 = 0 - up_m2
                if upi_m2 >100:
                    ui_m2 = 100 - up_m2
            upi_m2 = ui_m2  + up_m2
            print("upi_m motor 2= "+ str(upi_m2))

                    #Control esclavo para M1
            rk_s = upi_m
            yk_s = W
            ek_s= rk_s - yk_s
            iek_s = ek_s + iek_s_1
            ui_s = ki_s*iek_s
            up_s= kp_s*ek_s
            upi_s = up_s + ui_s
            if upi_s < 0 or upi_s > 100:
                if upi_s < 0 :
                    ui_s = 0 - up_s
                if upi_s >100:
                    ui_s = 100 - up_s
            upi_s = ui_s + up_s
            print("rks = "+ str(rk_s))
            print("pwm = "+ str(upi_s))
            print("flujo = "+ str(fm_n))


                        #Control esclavo para M2
            rk_s2 = upi_m2
            yk_s2 = W2
            ek_s2= rk_s2 - yk_s2
            iek_s2 = ek_s2 + iek_s_12
            ui_s2 = ki_s2*iek_s2
            up_s2= kp_s2*ek_s2
            upi_s2 = up_s2 + ui_s2
            if upi_s2 < 0 or upi_s2 > 100:
                if upi_s2 < 0 :
                    ui_s2 = 0 - up_s2
                if upi_s2 >100:
                    ui_s2 = 100 - up_s2
            upi_s2 = ui_s2 + up_s2
            print("rks2 = "+ str(rk_s2))
            print("pwm2 = "+ str(upi_s2))
            print("flujo2 = "+ str(fm_n2))

            motor_speed = upi_s  # Asegurar que motor1_speed esté en el rango 0-100
            control_motor(motor1_pwm_pin, motor1_dir_pin, motor_speed, 'forward')
            motor2_speed = upi_s2  # Asegurar que motor1_speed esté en el rango 0-100
            control_motor(motor2_pwm_pin, motor2_dir_pin, motor2_speed, 'forward')

    #       Reemplazo Variables m1
            delta_fn_2 = delta_fn_1
            delta_fn_1 = delta_fn
            delta_W_1 = delta_W
            iek_m_1 = iek_m
            iek_s_1 = iek_s
    #       Reemplazo Variables m2
            delta_fn_22 = delta_fn_12
            delta_fn_12 = delta_fn2
            delta_W_12 = delta_W2
            iek_m_12 = iek_m2
            iek_s_12 = iek_s2

            # Registro de datos
            ts = time.time() - start_time
            output_file.write(f"{ts:.2f}\t{upi_m:.2f}\t{upi_m2:.2f}\t{W1:.2f}\t{W2:.2f}\t{fm_n:.2f}\t{fm_n2:.2f}\n")

            # Restablecer contadores de flancos
            FLANCOS_M1_A=0.0
            FLANCOS_M1_B=0.0  
            FLANCOS_M1=0.0

            FLANCOS_M1_A2=0.0
            FLANCOS_M1_B2=0.0  
            FLANCOS_M2=0.0

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
