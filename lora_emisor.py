
#!/usr/bin/env python3
import serial, binascii
import time
import smbus2
import subprocess
from gpiozero import Motor
import numpy as np
from robot import Robot
from encoder import QuadratureEncoder
from gps import GPS
from bno055 import BNO055
from bme_280 import BME280Sensor
from ina226_sensor import INA226Sensor
from controller import PIDController
from manager import RoverManager
from sphericalTrigonometry import SphericalPoint
from calibration import Calibration
from gpiozero import OutputDevice
# --- Configuración RAK3172 ---
PORT = "/dev/serial0"
BAUD = 115200
P2P  = "915000000:7:0:0:16:20"   # Igual que en el receptor

# --- Funciones ---
def at(ser, cmd, wait=0.25, show=True):
    ser.write((cmd + "\r\n").encode()); ser.flush()
    time.sleep(wait)
    out = ser.read(ser.in_waiting or 1).decode(errors="ignore")
    time.sleep(0.05)
    out += ser.read(ser.in_waiting or 1).decode(errors="ignore")
    if show: #print(f"> {cmd}\n{out.strip()}\n")
    return out

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
# --- Main ---
with serial.Serial(PORT, BAUD, timeout=0.2) as s:
    LORA_RESET_PIN = 27  # Cambia este número si usas otro pin
    lora_reset = OutputDevice(LORA_RESET_PIN, active_high=True, initial_value=True)

    # Realiza el reset hardware
    lora_reset.off()  # Pone el pin en GND
    time.sleep(0.5)
    lora_reset.on()   # Lo pone en HIGH
    time.sleep(0.1)
  
    # Si no necesitas controlar motores desde aquí, puedes dejarlo en None
    bme280 = BME280Sensor()
    # Configurar módulo en P2P
    at(s, "AT")
    at(s, "AT+PRECV=0", show=False)
    at(s, "AT+NWM=0")
    at(s, f"AT+P2P={P2P}")
   
    #print("== Emisor BMP280 -> LoRa P2P ==")
    while True:
        try:
            data = bme280.read()
            payload = f"{data['temperature']}"
            #print("TX:", payload)
            psend(s, payload)
            time.sleep(1)
        except KeyboardInterrupt:
            break
