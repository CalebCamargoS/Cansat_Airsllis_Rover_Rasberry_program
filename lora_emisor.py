
#!/usr/bin/env python3
import serial, binascii
import time
from calibration import Calibration
from robot import Robot
from encoder import QuadratureEncoder
from gps import GPS
from bno055 import BNO055
from bme_280 import BME280Sensor
from ina226_sensor import INA226Sensor
from gpiozero import OutputDevice
# --- Configuración RAK3172 ---
PORT = "/dev/serial0"
BAUD = 115200
P2P  = "915000000:7:0:0:16:20"   # Igual que en el receptor

# --- Funciones ---
def at(ser, cmd, wait=0.25, show=True):
    ser.write((cmd + "\r\n").encode())
    ser.flush()
    time.sleep(wait)
    return None  # No leer respuesta

def psend(ser, text):
    hexpl = binascii.hexlify(text.encode("utf-8")).decode()
    return at(ser, f"AT+PSEND={hexpl}", wait=0.4)
def flatten_dict(d, parent_key='', sep='_'):
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep))
        else:
            items.append((new_key, v))
    return items

# Truncar decimales según tipo de dato
def truncate_value(key, value):
    if isinstance(value, float):
        if any(x in key for x in ["temperature", "pres", "hum"]):
            return round(value, 1)
        if any(x in key for x in ["latitude", "longitude"]):
            return round(value, 5)
        return round(value, 2)
    return value
# --- Main ---
with serial.Serial(PORT, BAUD, timeout=0.2) as s:
    LORA_RESET_PIN = 27  # Cambia este número si usas otro pin
    lora_reset = OutputDevice(LORA_RESET_PIN, active_high=True, initial_value=True)

    # Reset hardware LoRa
    lora_reset.off()
    time.sleep(0.5)
    lora_reset.on()
    time.sleep(0.1)

    # === Encoders ===
    LEFT_ENCODER_INPUT = {'hall_sensor_A':27, 'hall_sensor_B': 22, 'ticks_per_revolution': 985}
    RIGHT_ENCODER_INPUT = {'hall_sensor_A': 5, 'hall_sensor_B': 6, 'ticks_per_revolution': 985}
    left_encoder = QuadratureEncoder(
        LEFT_ENCODER_INPUT['hall_sensor_A'],
        LEFT_ENCODER_INPUT['hall_sensor_B'],
        LEFT_ENCODER_INPUT['ticks_per_revolution']
    )
    right_encoder = QuadratureEncoder(
        RIGHT_ENCODER_INPUT['hall_sensor_A'],
        RIGHT_ENCODER_INPUT['hall_sensor_B'],
        RIGHT_ENCODER_INPUT['ticks_per_revolution']
    )

    gps = GPS()
    bno055 = BNO055()
    bme280 = BME280Sensor()
    ina226 = INA226Sensor()

    robot = Robot(None, None, left_encoder, right_encoder, gps, bno055, bme280, ina226)
    calibration = Calibration(robot)

    # Configurar módulo en P2P
    at(s, "AT")
    at(s, "AT+PRECV=0", show=False)
    at(s, "AT+NWM=0")
    at(s, f"AT+P2P={P2P}")

    # Bucle principal
    while True:
        try:
            data = calibration.get_values()
            flat = flatten_dict(data)
            # Truncar valores y armar CSV
            values = [truncate_value(k, v) for k, v in flat]
            # Convertir a string y limitar tamaño (LoRa típico: 51-100 bytes)
            str_values = [str(v) for v in values]
            csv_payload = ','.join(str_values)
            # Si es muy largo, recortar (opcional: solo los primeros N campos)
            max_len = 90  # Ajusta según tu SF y payload permitido
            if len(csv_payload) > max_len:
                # Recorta campos hasta que quepa
                for i in range(len(str_values), 0, -1):
                    test_payload = ','.join(str_values[:i])
                    if len(test_payload) <= max_len:
                        csv_payload = test_payload
                        break
            psend(s, csv_payload)
            time.sleep(1)
        except KeyboardInterrupt:
            break
