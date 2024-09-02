import serial
import pynmea2

def main():
    port = "/dev/ttyAMA0"
    ser = serial.Serial(port, baudrate=9600, timeout=0.2)

    while True:
        try:
            # Lee datos del puerto serie, decodificados en UTF-8
            newdata = ser.readline().decode('utf-8').strip()

            # Imprime los datos leídos para depuración
            print(f"Raw data: {newdata}")

            # Procesa solo si la línea comienza con "$GPRMC"
            if newdata.startswith("$GPRMC"):
                try:
                    # Analiza el mensaje NMEA
                    newmsg = pynmea2.parse(newdata)
                    
                    # Obtiene latitud y longitud
                    lat = newmsg.latitude
                    lng = newmsg.longitude
                    print(f"Lat = {lat}, Lng = {lng}")

                    # Verifica y usa los atributos disponibles
                    if hasattr(newmsg, 'spd_over_grnd_kmph') and newmsg.spd_over_grnd_kmph is not None:
                        speed_kmph = newmsg.spd_over_grnd_kmph
                        print(f"Speed (km/h): {speed_kmph}")
                    else:
                        print("Speed data not available")
                    
                except pynmea2.ParseError as e:
                    print(f"Error parsing NMEA data: {e}")

        except serial.SerialException as e:
            print(f"Serial error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

if __name__ == "__main__":
    main()
