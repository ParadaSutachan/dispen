# -- coding: utf-8 --
import time
import pigpio

# Pines de motor y encoder
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

INTERVALO = 0.5  # Intervalo de tiempo en segundos

# Contadores de flancos
numero_flancos_A = 0
numero_flancos_B = 0
numero_flancos_A2 = 0
numero_flancos_B2 = 0

# Variables de tiempo
tiempo_anterior = 0
tiempo_anterior2 = 0

# Variables para RPM y RPS
RPS = 0.0
RPM = 0.0
RPS2 = 0.0
RPM2 = 0.0

# Inicialización de Pigpio
pi = pigpio.pi()

# Configuración de pines de entrada para los encoders
pi.set_mode(PIN_ENCODER_A, pigpio.INPUT)
pi.set_pull_up_down(PIN_ENCODER_A, pigpio.PUD_UP)
pi.set_mode(PIN_ENCODER_B, pigpio.INPUT)
pi.set_pull_up_down(PIN_ENCODER_B, pigpio.PUD_UP)

pi.set_mode(PIN_ENCODER2_A, pigpio.INPUT)
pi.set_pull_up_down(PIN_ENCODER2_A, pigpio.PUD_UP)
pi.set_mode(PIN_ENCODER2_B, pigpio.INPUT)
pi.set_pull_up_down(PIN_ENCODER2_B, pigpio.PUD_UP)

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
cb2 = pi.callback(PIN_ENCODER_B, pigpio.EITHER_EDGE, contador_flancos_encoder_b)
cb3 = pi.callback(PIN_ENCODER2_A, pigpio.EITHER_EDGE, contador_flancos_encoder2)
cb4 = pi.callback(PIN_ENCODER2_B, pigpio.EITHER_EDGE, contador_flancos_encoder_b2)

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

# Función principal
def main():
    global numero_flancos_A, numero_flancos_B, tiempo_anterior, numero_flancos_A2, numero_flancos_B2, tiempo_anterior2
    
    # Inicialización de tiempos
    tiempo_anterior = time.time()  
    tiempo_anterior2 = time.time()  
    
    # Habilitar motores
    pi.write(motor1_en_pin, 1)
    pi.write(motor2_en_pin, 1)

    # Ruta del archivo
    file_path = '/home/santiago/Documents/dispensador/dispensador/Pbrs1.txt'

    # Lectura de archivo
    with open(file_path, 'r') as file:
        lines = file.readlines()
        total_lines = len(lines)
        current_line1 = 0
        current_line2 = 0

        start_time = time.time()
        
        # Bucle principal
        while time.time() - start_time <= 100:  # Ejecutar durante 40 segundos
            tiempo_actual = time.time()
            tiempo_actual2 = time.time()

            line1 = lines[current_line1].strip()
            line2 = lines[current_line2].strip()
            
            # Obtener velocidades de los motores
            motor1_speed = int(line1)
            motor2_speed = int(line2)
            
            # Controlar los motores con las velocidades especificadas
            control_motor(motor1_pwm_pin, motor1_dir_pin, 50, 'forward')
            control_motor(motor2_pwm_pin, motor2_dir_pin, 100, 'forward')
            #
            # Avanzar en las líneas circularmente
            current_line1 = (current_line1 + 1) % total_lines
            current_line2 = (current_line2 + 1) % total_lines
            
            # Imprimir velocidades de los motores
            print('Velocidad motor 1:', motor1_speed)

        
            print('Velocidad motor 2:', motor2_speed)
            
            # Calcular RPS y RPM usando flancos contados
            tiempo_pasado = tiempo_actual - tiempo_anterior
            tiempo_pasado2 = tiempo_actual2 - tiempo_anterior2
            
            if tiempo_pasado >= INTERVALO:
                # Calcular RPS y RPM para el motor 1
                RPS = (numero_flancos_A + numero_flancos_B) / 1200.0
                RPM = RPS * 120.0
                
                print("Revoluciones por segundo M1: {:.4f} | Revoluciones por minuto M1: {:.4f}".format(RPS, RPM))

                # Calcular RPS y RPM para el motor 2
                print("flancos a",numero_flancos_A)
                print("flancos a",numero_flancos_B)
                RPS2 = (numero_flancos_A2 + numero_flancos_B2) / 1200.0
                RPM2 = RPS2 * 120.0
                
                print("Revoluciones por segundo M2: {:.4f} | Revoluciones por minuto M2: {:.4f}".format(RPS2, RPM2))

                # Restablecer contadores
                numero_flancos_B = 0
                numero_flancos_A = 0
                tiempo_anterior = tiempo_actual
                numero_flancos_B2 = 0
                numero_flancos_A2 = 0
                tiempo_anterior2 = tiempo_actual2

            time.sleep(0.5)

        # Deshabilitar motores
        pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
        pi.set_PWM_dutycycle(motor2_pwm_pin, 0)

        pi.write(motor1_en_pin, 0)
        pi.write(motor2_en_pin, 0)

        # Detener Pigpio
        pi.stop()
        print('Tiempo de funcionamiento de los motores completado.')

main()
