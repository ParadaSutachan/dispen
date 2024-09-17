# -- coding: utf-8 --
#!/usr/bin/env python3
import time
import pigpio #type: ignore
import math
from pytictoc import TicToc #type: ignore
import numpy as np # type: ignore
from numpy import array  #type: ignore
import serial #type:ignore
import RPi.GPIO as GPIO #type:ignore
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

# Configuración de pines
ESC_PIN_1 = 21  # Pin GPIO donde está conectado el primer ESC
ESC_PIN_2 = 20  # Pin GPIO donde está conectado el segundo ESC

GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PIN_1, GPIO.OUT)
GPIO.setup(ESC_PIN_2, GPIO.OUT)

# Configurar la señal PWM para los ESC (50 Hz)
pwm1 = GPIO.PWM(ESC_PIN_1, 50)  # Frecuencia de 50 Hz (20ms ciclo completo)
pwm2 = GPIO.PWM(ESC_PIN_2, 50)  # Frecuencia de 50 Hz (20ms ciclo completo)

T = 0.2
# Configuración de pines de entrada para los encoders
# Configuración de pines  
pin_a = 17  # Pin A del encoder  M1
pin_b = 18  # Pin B del encoder  M1
pin_a2 = 16 # Pin A del enconder M2
pin_b2 = 19 # Pin B del enconder M2

# Contadores de flancos

count = 0
count2 = 0  
last_state = 0
last_state2 = 0

def rotary_interrupt(channel):  
    global count  
    global last_state  

    if GPIO.input(pin_a) != last_state:  
        if GPIO.input(pin_b) != last_state:  
            count += 1  
        else:  
            count += 1  
    last_state = GPIO.input(pin_a)

def rotary_interrupt2(channel):  
    global count2  
    global last_state2  

    if GPIO.input(pin_a2) != last_state2:  
        if GPIO.input(pin_b2) != last_state2:  
            count2 += 1  
        else:  
            count2 += 1  
    last_state2 = GPIO.input(pin_a2)  

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

def set_speed(pwm, pulse_width):
    duty_cycle = pulse_width / 20000 * 100  # Convertir microsegundos a ciclo de trabajo
    pwm.ChangeDutyCycle(duty_cycle)

# Configuración GPIO  
GPIO.setmode(GPIO.BCM)  
GPIO.setup(pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.setup(pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.add_event_detect(pin_a, GPIO.BOTH, callback=rotary_interrupt)  
GPIO.setup(pin_a2, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.setup(pin_b2, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.add_event_detect(pin_a2, GPIO.BOTH, callback=rotary_interrupt2)  

# Matrices A,B,C,D del modelo
A = np.array([[0.894130801660748, 0.145973156319373, 0.066225686842812, 0.067086287188154],
            [-0.441836100194477, 0.748393868632439, 0.027872957540047, -0.026971019471969],
            [0.035223234234442, -0.404264770036147, 0.863341393359765, -0.539775953712378],
            [0.173570457083066, 0.507384555424424, 0.507384555424424, 0.507384555424424]])
B = np.array([[-0.000419785220888],
              [-0.006447149525435],
              [-0.003587244670196],
              [0.020003639959133]])
C = np.array([[-46.886951437617398, -8.778687090880192, 5.312400689483614, -2.660688977805191]])
D = 0 
# Matriz K y L y constante Ki 
#K = np.array([[-96.894235555587898, -54.978236188808395, -16.510278445545580, 0.225397518945667]])
# la que tenia al inicio K = np.array([[ -28.943823250907375,-14.916476691659597,  -5.284958871641084,  0.606037301182901]])
#K = np.array([[ -46.687992146378335, -25.234466885110013,  -8.383515963162855,   0.735588807482883]])
K = np.array([[ -39.182988561814568, -20.829192398061014,  -7.083573262725010,   0.700789477478013]]) #Mejor respuesta segun Santiago
#K = np.array([[ -41.629745839451004, -22.260482684956859,  -7.509364454424423,   0.715540695413975 ]])
#K = np.array([[ -40.836100468904952, -21.795633546054280,  -7.371481681852714,   0.711152516724570 ]])
#Ki = -0.344608368322768 # no puede ser mayor
Ki = -0.286889068064602  #best response
#Ki = -0.305710358015974
#Ki= -0.299605441621511
L = np.array([[-0.001633156413536],
              [0.000304322683582],
              [0.002212041259930],
              [-0.000008608682074]])
Ao = (A-L@C)
Bo = np.array([[-0.000419785220888, -0.001633156413536],
              [-0.006447149525435, 0.000304322683582],
              [-0.003587244670196, 0.002212041259930],
              [0.020003639959133, -0.000008608682074]])
#Definición de ek, rk, flujo y X

## variables error M1 ----------------------------------------------------------------------
ek = 0.0
ek_1 = 0.0
ek_int = 0.0
ek_int_1 = 0.0

## Variables error M2 ----------------------------------------------------------------------
ek2 = 0.0
ek2_1 = 0.0
ek2_int = 0.0
ek2_int_1 = 0.0

## Variables rk,fk,uk para M1 ---------------------------------------------------------------
rk = 0.0
fk = 0.0
uk = 0.0

##  Variables rk,fk,uk para M2 -----------------------------------------------------------------
rk2 = 0.0
fk2 = 0.0
uk2 = 0.0

## Variables set-point
F_b = 21.3465
W_b = 28

W = 0
W2 = 0

## Variables calculo de Soft sensor M1 -------------------------------------------------------------------
delta_w = 0.0
delta_w_1 = 0.0
delta_f = 0.0
delta_f_1 = 0.0 
delta_f_2 = 0.0

## Variables calculo de Soft Sensor M2 ---------------------------------------------------------------------
delta_w2 = 0.0
delta_w2_1 = 0.0
delta_f2 = 0.0
delta_f2_1 = 0.0 
delta_f2_2 = 0.0

k=0
gk=0
rate = 12.82
faja = 3.6
dosis_m1=0.0
dosis_m2=0.0

## Inicializacion variables para el calculo del observador ------------------------------------------------
xk = np.array([[0],
               [0],
               [0],
               [0]]) ## xk para M1

x2k = np.array([[0],
                [0],
                [0],
                [0]]) ## xk para M2


xk1 = np.array([[0],
                [0],
                [0],
                [0]]) ## xk1 para M1

x2k1 = np.array([[0],
                 [0],
                 [0],
                 [0]]) ## xk1 para M2

#Inicializacion del GPS
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
# Loop de Control
# Habilitar motores

pi.write(motor1_en_pin, 1)
pi.write(motor2_en_pin, 1)

# Inicializar PWM
pwm1.start(0)  # Iniciar con un duty cycle de 0%
pwm2.start(0)  # Iniciar con un duty cycle de 0%

# Crear el archivo de salida para guardar los datos
output_file_path = '/home/santiago/Documents/dispensador/dispen/beta-test.txt'
with open(output_file_path, 'w') as output_file:
    output_file.write("Tiempo \t PWM \t W \t Referencia \t Flujo \n")
    start_time = time.time()
    while(True):
        
        t1 = TicToc()       # Tic
        t1.tic()
        k += 1
        gk +=1

        if gk == 5:
            # Verifica si se recibe una sentencia GPRMC
            newdata = ser.readline().decode('utf-8').strip() 
             
            if newdata[0:6] == "$GPRMC":  
                newmsg = pynmea2.parse(newdata)  
                status = newmsg.status
                # read your shapefile
                poly_file='poligono_casona1.shp'
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

                    shapes = r.shapes()
                    inside_zone = False  # Bandera para verificar si está dentro de alguna zona
                    for j in range(len(shapes)):
                        # build a shapely polygon from your shape
                        polygon = shape(shapes[j])    
                        zone_def = check(lon, lat)
                        if zone_def: 
                            zona = j+1
                            if zona == 1:
                                dosis_m1 = 0.7*rate
                                dosis_m2 = 0.3*rate
                                #rk = 20
                                #rk2 = 35
                            elif zona == 2:
                                dosis_m1 = 0.4*rate
                                dosis_m2 = 0.6*rate
                                #rk = 35
                                #rk2 = 20
                            print('Estas en zona ' + str(zona))
                            inside_zone = True
                            break  # Sal del bucle si se encuentra una zona

                    if not inside_zone:
                        rk2 = 0
                        ek2 = 0
                        fk2 = 0
                        uk2 = 0
                        rk = 0
                        ek = 0
                        fk = 0
                        uk = 0
                        control_motor(motor2_pwm_pin, motor2_dir_pin, 0, 'forward')
                        control_motor(motor1_pwm_pin, motor1_dir_pin, 0, 'forward')
                        print("Estas Fuera de Rango . . .")

                elif status == "V":  
                    print("Buscando señal . . .")
            gk = 0

        set_speed(pwm1, 1200)  # Señal de 1200 microsegundos para el primer motor
        set_speed(pwm2, 1200)  # Señal de 1200 microsegundos para el segundo motor
        
        #Lectura de Flancos para medir velocidad
        FPS = count / (600.0)
        W = FPS * ((2 * pi_m) / T)      #Velocidad del motor
        #print("Velocidad M1: " + str(W))

        FPS2 = count2/(600.0)
        W2 = FPS2 *((2*pi_m)/T)
        #print("Velocidad M2: " + str(W2))

        # Soft Sensor M1 --------------------------------------------------------------------------
        delta_w = W-W_b
        delta_f = 0.1969*delta_w_1 + 1.359*delta_f_1 -0.581*delta_f_2

        if k <= 3:
             fk= 0
        else :
             fk=delta_f+F_b

        print("Flujo 1: " + str(fk))

        # Soft Sensor M2 ---------------------------------------------------------------------------

        delta_w2 = W2-W_b
        delta_f2 = 0.1969*delta_w2_1 + 1.359*delta_f2_1 -0.581*delta_f2_2

        if k <= 3:
             fk2= 0
        else :
             fk2=delta_f2+F_b

        print("Flujo 2: " + str(fk2))


        # Calulo de la referencia #
        #print(float(speed_mps))
        if speed_mps <= 0.3:
            speed_mps =0.0

        rk = speed_mps*dosis_m1*faja
        rk2 = speed_mps*dosis_m2*faja

        print("rk: " + str(rk))
        print("rk2: " + str(rk2))

        ##Observador ------------------------------------------------------------------------------------

        uo = np.array([[uk],
                       [fk]])
        
        u2o = np.array([[uk2],
                        [fk2]])
        
        xk1 = Ao@xk + Bo@uo

        x2k1 = Ao@x2k + Bo@u2o
        ## ------------------------------------------------------------------------------------------------

        ## Controlador M1 ---------------------------------------------------------------------------------
        ek = rk - fk
        print("ek: "+str(ek))

        ek_int = ek_1 + ek_int_1
        uik = ek_int*Ki
        ux_k= K@xk
        uk = -uik-float(ux_k[0]) #Accion de Control
        if uk < 0 or uk > 100:
            if uk < 0 :
                uik = 0 - float(ux_k[0])
            if uk >100:
                uik = -100 - float(ux_k[0])
        uk = -uik-float(ux_k[0])        
        motor1_speed = uk  
        print("uk = " + str(uk))

        ## COntrolador M2 --------------------------------------------------------------------------------------
        ek2 = rk2 - fk2
        print("ek2: "+str(ek2))

        ek2_int = ek2_1 + ek2_int_1
        uik2 = ek2_int*Ki
        ux2_k= K@x2k
        uk2 = -uik2-float(ux2_k[0])
        if uk2 < 0 or uk2 > 100:
            if uk2 < 0 :
                uik2 = 0 - float(ux2_k[0])
            if uk2 >100:
                uik2 = -100 - float(ux2_k[0])
        uk2 = -uik2-float(ux2_k[0])    
        motor2_speed = uk2  
        print("uk2 = " + str(uk2))

        control_motor(motor1_pwm_pin, motor1_dir_pin, motor1_speed, 'forward')
        control_motor(motor2_pwm_pin, motor2_dir_pin, motor2_speed, 'forward')
        
        ## Reasignacion de variables M1 -----------------------------------------------------------------------

        delta_f_2 = delta_f_1
        delta_f_1 = delta_f
        xk = xk1
        ek_int_1=ek_int
        ek_1 = ek
        delta_w_1 = delta_w

        delta_f2_2 = delta_f2_1
        delta_f2_1 = delta_f2
        x2k = x2k1
        ek2_int_1=ek2_int
        ek2_1 = ek2
        delta_w2_1 = delta_w2 

        # Registrar los datos en el archivo
        ts = time.time() - start_time
        output_file.write(f"{ts:.2f}\t{uk:.2f}\t{W:.2f}\t{rk} \t{fk:.2f}\n")
        output_file.flush()
        # Restablecer contadores
        count = 0
        count2 = 0
        e_time = t1.tocvalue()
        toc = abs(T-e_time)        #Toc
        time.sleep(toc)
    
# Deshabilitar motores
pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
pi.set_PWM_dutycycle(motor2_pwm_pin, 0)
pi.write(motor1_en_pin, 0)
pi.write(motor2_en_pin, 0)
# Detener Pigpio
pi.stop()
print('Tiempo de funcionamiento de los motores completado.')