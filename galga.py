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

# Inicialización de Pigpio
pi = pigpio.pi()
pi_m = math.pi 

#Puerto arduino

#arduino_port = '/dev/ttyACM1'  # Puerto donde está conectada la placa Arduino
#arduino_baud = 9600


# Configuración de pines de motor y encoder

motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23

T = 0.2

# Configuración de pines  
pin_a = 17  # Pin A del encoder  
pin_b = 18  # Pin B del encoder  

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
    
# Contadores de flancos

count = 0  
last_state = 0 
W = 0

def rotary_interrupt(channel):  
    global count  
    global last_state  

    if GPIO.input(pin_a) != last_state:  
        if GPIO.input(pin_b) != last_state:  
            count += 1  
        else:  
            count += 1  
    last_state = GPIO.input(pin_a)

# Configuración GPIO  
GPIO.setmode(GPIO.BCM)  
GPIO.setup(pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.setup(pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.add_event_detect(pin_a, GPIO.BOTH, callback=rotary_interrupt)  

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
#K = np.array([[ -39.182988561814568, -20.829192398061014,  -7.083573262725010,   0.700789477478013]]) #Mejor respuesta segun Santiago
#K = np.array([[ -41.629745839451004, -22.260482684956859,  -7.509364454424423,   0.715540695413975 ]])
#K = np.array([[ -36.195886071041144, -19.090018570665148 , -6.561829069892269,   0.679019386900889]]) #ward
K = np.array([[ -37.227496620872138, -19.689543163622165 , -6.742292356607224 ,  0.687066080921029]])


#Ki = -0.344608368322768 # no puede ser mayor
#Ki = -0.286889068064602  #best response
#Ki = -0.305710358015974
#Ki= -0.263922518365071 #ward
Ki= -0.271852325047363


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

ek = 0.0
ek_1 = 0.0
ek_int = 0.0
ek_int_1 = 0.0
rk = 0.0
fk = 0.0
W = 0.0
uk = 0.0
F_b = 21.3465
W_b = 28
delta_w = 0.0
delta_w_1 = 0.0
delta_f = 0.0
delta_f_1 = 0.0 
delta_f_2 = 0.0
k=0

xk = np.array([[0],
               [0],
               [0],
               [0]])

xk1 = np.array([[0], 
                   [0],
                   [0],
                   [0]]) 

#Inicializacion del arduino

#variables galga
#wg = 0.0
#GPIO.setwarnings(False)
#arduino = serial.Serial(arduino_port, arduino_baud)
#time.sleep(10)  # Esperar a que la conexión serial se establezca

#while True:
#    if arduino.in_waiting > 0:
#        mensaje = arduino.readline().decode('utf-8').strip()
#        if mensaje == "Listo para pesar":
#            print("Arduino ha completado la inicialización.")
#            break
#        else:
#            print(f"Mensaje de Arduino: {mensaje}")

# Loop de Control

# Habilitar motores
pi.write(motor1_en_pin, 1)
pi.write(motor2_en_pin, 1)
rk=20

# Crear el archivo de salida para guardar los datos
output_file_path = '/home/santiago/Documents/dispensador/dispen/test_control_ss.txt'


with open(output_file_path, 'w') as output_file:
    output_file.write("Tiempo \t PWM \t W \t Referencia \tFlujo \t Peso \n")

    #wg = arduino.readline().decode('utf-8')
    #print("Peso: " +str(wg))

    start_time = time.time()

    while(time.time()-start_time <= 60):
        
        
        t1 = TicToc()       # Tic
        t1.tic()
        k += 1

        #Lectura de Flancos para medir velocidad
        FPS = count / (600.0)
        W = FPS * ((2 * pi_m) / T)      #Velocidad del motor
        print("Velocidad: " + str(W))
        # Soft Sensor
        delta_w = W-W_b
        delta_f = 0.1969*delta_w_1 + 1.359*delta_f_1 -0.581*delta_f_2

        if k <= 3:
             fk= 0
        else :
             fk=delta_f+F_b
        ##

        if k == 100:
            rk=40
        if k == 200:
            rk =30

        ##Observador
        uo = np.array([[uk],
                       [fk]])
        
        xk1 = Ao@xk + Bo@uo
        ##

        ## Controlador
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

        control_motor(motor1_pwm_pin, motor1_dir_pin, motor1_speed, 'forward')
        
        delta_f_2 = delta_f_1
        delta_f_1 = delta_f
        xk = xk1
        ek_int_1=ek_int
        ek_1 = ek
        delta_w_1 = delta_w 

        # Medir peso
        #if arduino.in_waiting > 0:
        #   wg = arduino.readline().decode('utf-8')
        #    print("Peso: " +str(wg))

        # Registrar los datos en el archivo
        ts = time.time() - start_time
        output_file.write(f"{ts:.2f}\t{uk:.2f}\t{W:.2f}\t{rk} \t{fk:.2f}\t\n")
        output_file.flush()

        # Restablecer contador Flancos
        
        count = 0

        print("Flujo = "+ str(fk))

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