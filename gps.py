
from serial import Serial
from sphericalTrigonometry import SphericalPoint
import time
import threading

class GPS:
    GGA_TYPE = 'GGA'

    def __init__(self, port="/dev/serial0", baud=9600, timeout=1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.serial=Serial(port, baud, timeout=timeout)
        self.last_point = SphericalPoint(0.0, 0.0)
        self.last_alt = 0.0
        self._stop_thread = False
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

    def _update_loop(self):
        while not self._stop_thread:
            try:
                point, alt = self.read(blocking=True)
                self.last_point = point
                self.last_alt = alt
            except Exception:
                pass
            # No sleep: el GPS es lento por sí mismo

    def parse_nmea_sentence(self, sentence):
        parsed_sentence = {}
        values = sentence.split(',')
        parsed_sentence['type'] = values[0][3:] if len(values[0]) >= 6 else ""

        if parsed_sentence['type'] == self.GGA_TYPE and len(values) >= 10:
            # Latitude
            if values[2]:
                latitude = int(values[2][:2]) + float(values[2][2:]) / 60.0
                if values[3] == 'S':
                    latitude = -latitude
                parsed_sentence['latitude'] = latitude

            # Longitude
            if values[4]:
                longitude = int(values[4][:3]) + float(values[4][3:]) / 60.0
                if values[5] == 'W':
                    longitude = -longitude
                parsed_sentence['longitude'] = longitude

            # Altitude
            try:
                parsed_sentence['altitude'] = float(values[9]) if values[9] else None
            except ValueError:
                parsed_sentence['altitude'] = None

        return parsed_sentence

    def read(self):
        parsed_sentence = {}
        while True:
            try:
                data = self.serial.readline()
                sentence = data.decode('utf-8', errors='ignore').strip()
                if not sentence.startswith("$"):
                    continue

                parsed_sentence = self.parse_nmea_sentence(sentence)

                if parsed_sentence.get('type') == self.GGA_TYPE:
                    break
            except Exception:
                pass

        lat = parsed_sentence.get('latitude', 0.0)
        lon = parsed_sentence.get('longitude', 0.0)
        alt = parsed_sentence.get('altitude', 0.0) or 0.0
        try:
            with open("gps_data.txt", "w") as f:
                f.write(f"{lat},{lon}\n")
        except Exception:
            pass

        return SphericalPoint(lat, lon), alt

    def stop(self):
        self._stop_thread = True
        if self._thread.is_alive():
            self._thread.join()


if __name__ == '__main__':
    gps = GPS(port="/dev/serial0")  

    #print("Reading GPS data... (Ctrl+C to stop)")
    reference_point = None

    while True:
        try:
            point, altitude = gps.read()
            #print("(lat,long):","(",point.latitude,",",point.longitude,")")
            """
            if reference_point is None:
                # Primer punto como referencia
                reference_point = SphericalPoint(point.latitude, point.longitude)
                ref_alt = altitude
                print(f"Reference set at Lat={point.latitude:.7f}, Lon={point.longitude:.7f}, Alt={altitude:.2f}m")
                continue
            
            # Convertir a coordenadas locales ENU respecto al punto inicial
            enu = point.toENU(reference_point)
            x, y, z = enu[0], enu[1], (altitude - ref_alt)  # z = diferencia de altura

            msg = f"Local ENU -> x={x:.2f} m (East), y={y:.2f} m (North), z={z:.2f} m (Up)"
            print(msg)

            time.sleep(1)
            """
        except KeyboardInterrupt:
            print("\nStopped by user.")
            break
        except Exception as e:
            print("Error:", e)
            
