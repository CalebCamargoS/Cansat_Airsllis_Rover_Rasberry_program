
import math
import numpy as np
from bno055 import BNO055
from bme_280 import BME280Sensor
from ina226_sensor import INA226Sensor
from motor import Motor
from encoder import QuadratureEncoder
from gps import GPS
import time
from sphericalTrigonometry import *
class Robot():
    def __init__(self, left_motor, right_motor, left_encoder, right_encoder, gps, bno055, bme280, ina226):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.speed = 0
        self.w = 0
        self.wheel_radius = 0.08
        self.wheel_base_length = 0.19
        self.max_left_wheel_speed = 10
        self.max_right_wheel_speed = 10
        self.max_speed = 10

        self.left_motor = left_motor
        self.right_motor = right_motor
        self.left_encoder = left_encoder
        self.right_encoder = right_encoder
        self.gps = gps
        self.bno055 = bno055
        self.bme280 = bme280
        self.ina226 = ina226  

        self.reference, _ = self.gps.read()
        self.theta = bno055.get_heading_radians()
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.stop()
    #ok
    def forward(self, speed = 1):
        self.left_motor.forward(speed)
        self.right_motor.forward(speed)
    #ok
    def backward(self, speed = 1):
        self.left_motor.backward(speed)
        self.right_motor.backward(speed)
    #ok
    def update_speed(self, left_wheel_speed, right_wheel_speed):#velocidad absoluta, este usan en maanger
        left_speed = left_wheel_speed / self.max_left_wheel_speed
        right_speed = right_wheel_speed / self.max_right_wheel_speed
        if left_speed < 0:
            self.left_motor.backward(-left_speed)
        else:
            self.left_motor.forward(left_speed)
        if right_speed < 0:
            self.right_motor.backward(-right_speed)
        else:
            self.right_motor.forward(right_speed)
    #ok
    def update_speed_normalize(self, left_speed, right_speed):#en porcentaje y ambos positivos
        if left_speed < 0:
            self.left_motor.backward(-left_speed)
        else:
            self.left_motor.forward(left_speed)
        if right_speed < 0:
            self.right_motor.backward(-right_speed)
        else:
            self.right_motor.forward(right_speed)

    def stop(self):#ok
        self.left_motor.stop()
        self.right_motor.stop()

    def get_state(self):#ok
        return np.array([
            [self.x],
            [self.y],
            [self.theta],
            [self.speed],
            [self.w]
        ])

    def get_environment(self):#ok
        return self.bme280.read()

    def get_battery_status(self):#ok
        return self.ina226.read_voltage()
    
    def active_calibration_bno055(self, timeout=120, speed=0.3):
        """
        Ejecuta un movimiento en forma de '8' hasta que el BNO055 esté calibrado.
        speed: velocidad normalizada (0.0 - 1.0)
        timeout: tiempo máximo de calibración en segundos
        """
        print("\n[CALIBRATION] Iniciando calibración activa del BNO055...")
        start_time = time.time()

        # Bucle principal de calibración
        while time.time() - start_time < timeout:
            sys, gyro, accel, mag = self.bno055.get_calibration_status()
            print(f"SYS={sys} GYRO={gyro} ACCEL={accel} MAG={mag}", end="\r")

            # Verificar si ya está completamente calibrado
            if sys >= 0 and gyro >= 0 and accel >= 0 and mag == 3:
                print("\n✅ BNO055 calibrado completamente durante el movimiento!")
                self.stop()
                return True

            # Movimiento en forma de 8: primero giro a la izquierda, luego a la derecha
            self.update_speed_normalize(speed, speed /3)   # curva izquierda
            time.sleep(2)
            self.update_speed_normalize(speed /3, speed)   # curva derecha
            time.sleep(2)
            self.update_speed_normalize(speed,speed)
            time.sleep(2)
            self.update_speed_normalize(-speed,-speed)
            time.sleep(2)
        # Si no se calibró en el tiempo límite
        self.stop()
        print("\n⚠ Timeout: no se logró calibrar completamente el BNO055.")
        

    def get_full_state(self):#Ok
        ambiente=self.get_environment()
        gps_val=self.gps.read()
        return {
            "pose": {
                "x": self.x,
                "y": self.y,
                "theta": self.theta,
                "speed": self.speed,
                "angular_speed": self.w
            },
            "temperature": ambiente["temperature"],
            "pressure": ambiente["pressure"],
            "humidity" : ambiente["humidity"],
            "battery": self.get_battery_status(),
            "gps": { 
		"longitude":gps_val[0].longitude,
		"latitude":gps_val[0].latitude,
		"altitude":gps_val[1]
	    } 
        }
    #ok
    def set_state(self, state):
        self.x = state[0, 0]
        self.y = state[1, 0]
        self.theta = math.atan2(math.sin(state[2, 0]), math.cos(state[2, 0]))
        self.speed = state[3, 0]
        self.w = state[4, 0]


if __name__ == "__main__":

    # --- Initialize hardware ---
    left_motor = Motor(18,12)   # adjust pins!
    right_motor = Motor(13,19)

    left_encoder = QuadratureEncoder(tpr=985,
                                     pin_a=27, pin_b=22)
    right_encoder = QuadratureEncoder(tpr=985,
                                      pin_a=5, pin_b=6)

    gps = GPS()  # your GPS driver (or mock if not ready yet)

    bno055 = BNO055()
    bme280 = BME280Sensor()
    ina226 = INA226Sensor()

    # --- Create robot instance ---
    robot = Robot(left_motor, right_motor,
                  left_encoder, right_encoder,
                  gps, bno055, bme280, ina226)
    """
    print("=== ROBOT TEST START ===")
    print("Initial state:", robot.get_full_state())

    # --- Test motors + encoders ---
    print("\n[TEST] Moving forward...")
    robot.left_motor.forward(50)
    robot.right_motor.forward(50)
    time.sleep(2)
    robot.stop()

    print("Left encoder ticks:", left_encoder.ticks())
    print("Right encoder ticks:", right_encoder.ticks())

    # --- Test sensors individually ---
    print("\n[TEST] Orientation (BNO055)")
    print("Heading [rad]:", robot.bno055.get_heading_radians())


    print("\n[TEST] Environment (BME280)")
    print(robot.get_environment())

    print("\n[TEST] Battery Voltage (INA226)")
    print(robot.get_battery_status())
    longitud_latitude, altitude = robot.gps.read()
    print("\n[TEST] GPS Position")
    print("longitude:",longitud_latitude.longitude)
    print("latitude:",longitud_latitude.latitude)
    print("altitude:",altitude)

    # --- Test backward movement ---
    print("\n[TEST] Moving backward...")
    robot.left_motor.backward(50)
    robot.right_motor.backward(50)
    time.sleep(2)
    robot.stop()

    print("Left encoder ticks:", left_encoder.ticks())
    print("Right encoder ticks:", right_encoder.ticks())

    print("\nFinal full state:", robot.get_full_state())
    print("=== ROBOT TEST END ===")
    """
    # ...inicialización de robot y sensores como ya tienes...

    # Define el objetivo
    target = SphericalPoint(-12.02460, -77.047482)  # Cambia por tu objetivo real

    while True:
        # Obtén la posición actual del GPS
        current_point, _ = robot.gps.read()  # Asegúrate que tu GPS.read() retorna SphericalPoint como primer elemento

        # Calcula el bearing hacia el objetivo
        bearing = current_point.bearingTo(target)
        bearing = math.atan2(math.sin(bearing),math.cos(bearing))
        bearing_deg = math.degrees(bearing)

        # Lee el heading del BNO055
        heading_rad = robot.bno055.get_heading_radians()
        heading_rad = math.atan2(math.sin(heading_rad),math.cos(heading_rad))
        heading_deg = math.degrees(heading_rad)

        print(f"Bearing hacia objetivo: {bearing:.3f} rad ({bearing_deg:.2f}°)")
        print(f"Heading BNO055: {heading_rad:.3f} rad ({heading_deg:.2f}°)")
        print("-" * 40)
        time.sleep(1)
