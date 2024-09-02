import serial
import pynmea2

def main():
    port = "/dev/ttyAMA0"  # Ajusta el puerto según tu configuración
    ser = serial.Serial(port, baudrate=9600, timeout=0.2)

    while True:
        try:
            # Lee datos del puerto serie, decodificados en UTF-8
            newdata = ser.readline().decode('utf-8').strip()

            # Imprime los datos leídos para depuración
            print(f"Raw data: {newdata}")

            # Procesa mensajes $GPRMC
            if newdata.startswith("$GPRMC"):
                try:
                    newmsg = pynmea2.parse(newdata)
                    lat = newmsg.latitude
                    lng = newmsg.longitude
                    #print(f"Lat = {lat}, Lng = {lng}")
                except pynmea2.ParseError as e:
                    print(f"Error parsing $GPRMC data: {e}")

            # Procesa mensajes $GPVTG para obtener la velocidad
            elif newdata.startswith("$GPVTG"):
                try:
                    newmsg = pynmea2.parse(newdata)
                    if newmsg.spd_over_grnd_kmph is not None:
                        speed_kmph = newmsg.spd_over_grnd_kmph
                        speed_mps = speed_kmph * (1000 / 3600)
                     #   print(f"Speed (horizontal): {speed_mps:.2f} m/s")
                    else:
                        print("Speed data not available in $GPVTG")
                except pynmea2.ParseError as e:
                    print(f"Error parsing $GPVTG data: {e}")

        except serial.SerialException as e:
            print(f"Serial error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

if __name__ == "__main__":
    main()
