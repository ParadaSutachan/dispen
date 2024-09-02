
import serial
import pynmea2

def main():
    port = "/dev/ttyAMA0"
    ser = serial.Serial(port, baudrate=9600, timeout=0.5)
    dataout = pynmea2.NMEAStreamReader()

    while True:
        try:
            # Lee datos del puerto serie, decodificados en UTF-8
            newdata = ser.readline().decode('utf-8', errors='replace').strip()

            # Imprime los datos leídos para depuración
            print(f"Raw data: {newdata}")
            
            # Procesa solo si la línea comienza con "$GPRMC"
            if newdata.startswith("$GPRMC"):
                try:
                    # Analiza el mensaje NMEA
                    newmsg = pynmea2.parse(newdata)
                    if newmsg.spd_over_grnd_kmph is not None:
                        # Convierte km/h a m/s
                        speed_kmph = newmsg.spd_over_grnd_kmph
                        speed_mps = speed_kmph * (1000 / 3600)
                        print(f"Speed (horizontal): {speed_mps:.2f} m/s")
                    else:
                        print("Speed data not available")
                except pynmea2.ParseError as e:
                    print(f"Error parsing NMEA data: {e}")
        except serial.SerialException as e:
            print(f"Serial error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

    main()