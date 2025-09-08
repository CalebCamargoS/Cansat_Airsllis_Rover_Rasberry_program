
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
        if any(x in key for x in ["latitude", "longitude"]):
            return round(value, 5)
        return round(value, 2)
    # Si es una tupla, trunca cada elemento
    if isinstance(value, tuple):
        return tuple(truncate_value(key, v) for v in value)
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

    import os
    DATA_FILE = "data_to_send.txt"
    # Configurar módulo en P2P
    at(s, "AT")
    at(s, "AT+PRECV=0", show=False)
    at(s, "AT+NWM=0")
    at(s, f"AT+P2P={P2P}")

    # Bucle principal: lee el archivo y envía la última línea
    while True:
        try:
            if os.path.exists(DATA_FILE):
                with open(DATA_FILE, 'r') as f:
                    lines = f.readlines()
                if lines:
                    # Procesar y truncar los valores de las tuplas correctamente
                    raw = lines[-1].strip()
                    # Convertir string CSV a lista de valores
                    import ast
                    def process_value(val):
                        try:
                            v = ast.literal_eval(val)
                            if isinstance(v, tuple):
                                # Truncar cada elemento
                                return '(' + ','.join(f'{(round(x,5) if i<2 and k in ["latitude","longitude"] else round(x,2)) if isinstance(x,float) else x}' for i, x in enumerate(v)) + ')'
                            return round(v,5) if k in ["latitude","longitude"] and isinstance(v,float) else (round(v,2) if isinstance(v,float) else v)
                        except:
                            return val
                    # Reconstruir el payload truncando tuplas
                    values = raw.split(',')
                    new_values = []
                    for v in values:
                        v = v.strip()
                        if v.startswith('(') and v.endswith(')'):
                            # Truncar cada elemento de la tupla
                            try:
                                t = ast.literal_eval(v)
                                t2 = tuple(round(x,2) if isinstance(x,float) else x for x in t)
                                new_values.append(str(t2).replace(' ',''))
                            except:
                                new_values.append(v)
                        else:
                            try:
                                f = float(v)
                                new_values.append(f'{f:.5f}' if len(new_values)<2 else f'{f:.2f}')
                            except:
                                new_values.append(v)
                    payload = ','.join(new_values)
                    max_len = 180
                    if len(payload) > max_len:
                        payload = payload[:max_len]
                    psend(s, payload)
            time.sleep(1)
        except KeyboardInterrupt:
            break
