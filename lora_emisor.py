import time
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
def flatten_dict(d, parent_key='', sep='_'):
        items = []
        for k, v in d.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else k
            if isinstance(v, dict):
                items.extend(flatten_dict(v, new_key, sep=sep))
            else:
                items.append((new_key, v))
        return items
if __name__ == "__main__":
    # Configura el pin GPIO que controla el reset del RAK3172
    LORA_RESET_PIN = 27  # Cambia este número si usas otro pin
    lora_reset = OutputDevice(LORA_RESET_PIN, active_high=True, initial_value=True)

    # Realiza el reset hardware
    lora_reset.off()  # Pone el pin en GND
    time.sleep(0.5)
    lora_reset.on()   # Lo pone en HIGH
    time.sleep(0.1)

    # Inicializa el objeto Calibration
    # Inicializa el mismo robot que en main.py
    LEFT_MOTOR_INPUT = (18, 12)
    RIGHT_MOTOR_INPUT = (13, 19)
    left_motor = Motor(LEFT_MOTOR_INPUT[0], LEFT_MOTOR_INPUT[1])
    right_motor = Motor(RIGHT_MOTOR_INPUT[0], RIGHT_MOTOR_INPUT[1])
    # Si no necesitas controlar motores desde aquí, puedes dejarlo en None
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
    robot = Robot(left_motor, right_motor, left_encoder, right_encoder, gps, bno055, bme280, ina226)
    calibration = Calibration(robot)
    

    data = calibration.get_values()
    flat = flatten_dict(data)
    payload = ','.join(str(v) for _, v in flat)
    print(payload) 