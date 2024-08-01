import serial
import time
from pytictoc import TicToc

t = TicToc()


arduino_port = '/dev/ttyACM0'  # Puerto donde está conectada la placa Arduino
arduino_baud = 9600

arduino = serial.Serial(arduino_port, arduino_baud)
time.sleep(2)  # Esperar a que la conexión serial se establezca

start_time = time.time()

while time.time() - start_time <= 10:
    try:
        print("Enviando comando de calibración...")
        arduino.write(b'C')  # Enviar comando de calibración a Arduino
        t.tic()
        response = arduino.readline().decode('utf-8').rstrip()

        print(response)

        t.toc()
        time.sleep(0.2)  # Esperar un segundo antes de volver a enviar el comando
        
    except KeyboardInterrupt:
        print("Interrupción de teclado. Programa detenido.")

arduino.close()
