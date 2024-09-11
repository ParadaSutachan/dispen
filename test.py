# -- coding: utf-8 --
#!/usr/bin/env python3
import time
import pigpio
import RPi.GPIO as GPIO
import math
import serial
from pytictoc import TicToc
import pynmea2 #type: ignore
import shapefile #type: ignore
from shapely.geometry import shape, Point   #type: ignore
# Inicialización de Pigpio
pi = pigpio.pi()
pi_m = math.pi
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
# Configuración de pines de entrada para los encoders
# Configuración de los pines A y B con pull-up y detección de ambos flancos
def rotary_interrupt(channel):

    global FLANCOS_M1_A, FLANCOS_M1_B, FLANCOS_M1,FLANCOS_M1_A2,FLANCOS_M1_B2
    
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


GPIO.setup(PIN_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_ENCODER2_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN_ENCODER2_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Añadir detección de eventos en ambos pines para detectar flancos ascendentes y descendentes
GPIO.add_event_detect(PIN_ENCODER_A, GPIO.BOTH, callback=rotary_interrupt)
GPIO.add_event_detect(PIN_ENCODER_B, GPIO.BOTH, callback=rotary_interrupt)
GPIO.add_event_detect(PIN_ENCODER2_A, GPIO.BOTH, callback=rotary_interrupt)
GPIO.add_event_detect(PIN_ENCODER2_B, GPIO.BOTH, callback=rotary_interrupt)

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
# Configuración de callbacks
cb1 = pi.callback(PIN_ENCODER_A, pigpio.EITHER_EDGE, contador_flancos_encoder)
cb3 = pi.callback(PIN_ENCODER2_A, pigpio.EITHER_EDGE, contador_flancos_encoder2)
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
def check(lon, lat):
    # build a shapely point from your geopoint
    point = Point(lon, lat)
    # the contains function does exactly what you want
    return polygon.contains(point)
while True:
    newdata = ser.readline().decode('utf-8').strip()  
    
    # Verifica si se recibe una sentencia GPRMC  
    if newdata[0:6] == "$GPRMC":  
        newmsg = pynmea2.parse(newdata)  
        status = newmsg.status   
        # Maneja los estados A y V  
        if status == "A":  
            lat = newmsg.latitude  
            lon = newmsg.longitude  
            gps = f"Lat = {lat} Lng = {lon}"  
            print(gps)  
            speed = newmsg.spd_over_grnd  # velocidad en nudos  
            speed_mps = speed * (0.514444)  # convertimos de nudos a m/s  
            print(f"Speed: {speed:.2f} knots / {speed_mps:.2f} m/s")
            break
        elif status == "V":  
            print("Buscando señal . . .")

speed_mps=0.0
d=3.6
zona=0
gk=0.0
dosis_m1= 0.0
dosis_m2 = 0.0


# Loop de Control
start_time = time.time()
rk_m= float(speed_mps*d*rate)
rk_m2= float(speed_mps*d*rate) #  M2
# Habilitar motores
pi.write(motor1_en_pin, 1)
pi.write(motor2_en_pin, 1)
delta_W_1= 0.0
delta_fn_1 =0.0
delta_fn_2 = 0.0
contador=0
delta_W_12= 0.0
delta_fn_12 =0.0
delta_fn_22 = 0.0
# Crear el archivo de salida para guardar los datos
output_file_path = '/home/santiago/Documents/dispensador/dispen/uwurrr.txt'
with open(output_file_path, 'w') as output_file:
    output_file.write("Tiempo \t PWM_1 \t PWM_M2 \t W1 \t W2 \tFlujo \t Flujo2\n")
    start_time = time.time()
    while(True):
        
        t1 = TicToc()       # Tic
        t1.tic()
        k += 1
        gk +=1
        if gk == 3:
            newdata = ser.readline().decode('utf-8').strip()
            if newdata[0:6] == "$GPRMC":
                newmsg = pynmea2.parse(newdata)  
                status = newmsg.status
                # read your shapefile
                poly_file='poligono_casona.shp'
                r = shapefile.Reader(poly_file)
                # Maneja los estados A y V
                if status == "A":
                    lat = newmsg.latitude
                    lon = newmsg.longitude
                    gps = f"Lat = {lat} Lng = {lon}"
                    print(gps)
                    speed = newmsg.spd_over_grnd  # velocidad en nudos
                    speed_mps = speed * (0.514444)  # convertimos de nudos a m/s  
                    print(f"Speed: {speed_mps:.2f} m/s")
                    # get the shapes
                    shapes = r.shapes()
                    inside_zone = False # Bandera para verificar si está dentro de alguna zona
                    for j in range(len(shapes)):
                        polygon = shape(shapes[j])
                        zone_def = check(lon,lat)
                        if zone_def:
                            zone = j
                            zona = zone+1
                            inside_zone = True
                            if zona == 1:
                                dosis_m1 = float(0.7*rate)
                                dosis_m2=  float(0.3*rate)
                                print(("dosis m1= ") + str(dosis_m1))
                                print(("dosis m2= ") + str(dosis_m2))
                            if zona == 2:
                                dosis_m1 = float(0.4*rate)
                                dosis_m2=  float(0.6*rate)
                                print(("dosis m1= ") + str(dosis_m1))
                                print(("dosis m2= ") + str(dosis_m2))
                            break
                    
                    while not inside_zone:
                        print("Estas Fuera del Aerea a implementar . . .")
                        rk_m = 0.0
                        rk_m2 = 0.0
                        control_motor(motor1_pwm_pin, motor1_dir_pin, 0, 'forward')
                        newdata = ser.readline().decode('utf-8').strip()
                        if newdata[0:6] == "$GPRMC":
                            newmsg = pynmea2.parse(newdata)  
                            status = newmsg.status
                            # read your shapefile
                            poly_file='poligono_casona.shp'
                            r = shapefile.Reader(poly_file)
                            # Maneja los estados A y V
                            if status == "A":
                                lat = newmsg.latitude
                                lon = newmsg.longitude
                                gps = f"Lat = {lat} Lng = {lon}"
                                print(gps)
                                # get the shapes
                                shapes = r.shapes()
                                inside_zone = False # Bandera para verificar si está dentro de alguna zona
                                for j in range(len(shapes)):
                                    polygon = shape(shapes[j])
                                    zone_def = check(lon,lat)
                                    if zone_def:
                                        zone = j
                                        zona = zone+1
                                        inside_zone = True
                                    if zona == 1:
                                        dosis_m1 = float(0.7*rate)
                                        dosis_m2=  float(0.3*rate)
                                        print(("dosis m1= ") + str(dosis_m1))
                                        print(("dosis m2= ") + str(dosis_m2))
                                    if zona == 2:
                                        dosis_m1 = float(0.4*rate)
                                        dosis_m2=  float(0.6*rate)
                                        print(("dosis m1= ") + str(dosis_m1))
                                        print(("dosis m2= ") + str(dosis_m2))
                                        break
                        time.sleep(0.2)
            gk=0
        
        rk_m = float(d*speed_mps*dosis_m1)
        rk_m2 = float(d*speed_mps*dosis_m2)

        # Calcular FPS y W
        FPS = FLANCOS_M1 / 1200.0
        W = FPS*((2 * pi_m) / 0.2)
        print("W: " + str(W))


        # Calcular FPS y W
        FPS2 = FLANCOS_M2 / 1200.0
        W = FPS2*((2 * pi_m) / 0.2)
        print("W2: " + str(W))


        #Msoft sensor 
        delta_W = W - setpoint_W
        delta_fn= 0.1969*delta_W_1 + 1.359 * delta_fn_1 - 0.581*delta_fn_2 
        if k <= 3:
            fm_n= 0
        else :
            fm_n = delta_fn + setpoint_f            # PARA M1
#-------------------------------------------------------------------------------------------------
        delta_W2 = W2 - setpoint_W
        delta_fn2= 0.1969*delta_W_12 + 1.359 * delta_fn_12 - 0.581*delta_fn_22 
        if k2 <= 3:
            fm_n2= 0
        else :                                                          # Softsensor PARA M2
            fm_n2 = delta_fn2 + setpoint_f
#----------------------------------------------------------------------------------------------------
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
        control_motor(motor1_pwm_pin, motor1_dir_pin, 100, 'forward')
        motor2_speed = upi_s2  # Asegurar que motor1_speed esté en el rango 0-100
        control_motor(motor2_pwm_pin, motor2_dir_pin, 50, 'forward')
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
        
        # Registrar los datos en el archivo
        ts = time.time() - start_time
        output_file.write(f"{ts:.2f}\t{upi_s:.2f}\t{upi_s2:.2f}\t{W:.2f}\t{W2:.2f}\t{fm_n}\t{fm_n2:.2f}")
        # Restablecer contadores
        FLANCOS_M1_A=0.0
        FLANCOS_M1_B=0.0  
        FLANCOS_M1=0.0

        FLANCOS_M1_A2=0.0
        FLANCOS_M1_B2=0.0  
        FLANCOS_M2=0.0


        e_time= t1.tocvalue()
        toc= abs(INTERVALO- e_time)
        time.sleep(toc)
        
        
    
# Deshabilitar motores
pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
pi.set_PWM_dutycycle(motor2_pwm_pin, 0)
pi.write(motor1_en_pin, 0)
pi.write(motor2_en_pin, 0)
# Detener Pigpio
pi.stop()
print('Tiempo de funcionamiento de los motores completado.')