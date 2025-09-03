import time
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

def main():
    dt = 0.01   # 100 Hz (puedes ajustar según tu loop)
    min_altitude_air=10 # altitude to know if the cansat was launched
    alt_ref=0# referential altitude when it's in the land
    # === Motores ===
    LEFT_MOTOR_INPUT = (18, 12)
    RIGHT_MOTOR_INPUT = (13, 19)
    left_motor = Motor(LEFT_MOTOR_INPUT[0], LEFT_MOTOR_INPUT[1])
    right_motor = Motor(RIGHT_MOTOR_INPUT[0], RIGHT_MOTOR_INPUT[1])

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

    robot = Robot(left_motor, right_motor, left_encoder, right_encoder, gps, bno055, bme280, ina226)
    controller = PIDController(robot)

 
    target = SphericalPoint(-12.02460, -77.047482)
    rover_manager = RoverManager(robot, controller, target)
    calibration=Calibration(robot)
    tasks=["sensorCalibration",
           "inAir",
           "nicrom",
           "GPSControl",
           "CamaraControl"]
    
    currently_task=tasks[2]
    epoch = 0
    try:
        """
        N_REF = 200  # Número de muestras para referencia
        gps_altitudes = []
        bme_altitudes = []

        print("Calculando referencia de altitud...")

        for _ in range(N_REF):
            values = calibration.get_values()
            gps_alt = values["gps"]["altitude_gps"]
            bme_alt = values["environment"]["altitude_bme280"]
            if gps_alt is not None:
                gps_altitudes.append(gps_alt)
            if bme_alt is not None:
                bme_altitudes.append(bme_alt)
            time.sleep(0.01)  # 10 ms entre muestras

        alt_ref_gps = np.mean(gps_altitudes) if gps_altitudes else 0
        alt_ref_bme = np.mean(bme_altitudes) if bme_altitudes else 0

        print(f"Referencia de altitud GPS (media de {len(gps_altitudes)}): {alt_ref_gps:.2f} m")
        print(f"Referencia de altitud BME280 (media de {len(bme_altitudes)}): {alt_ref_bme:.2f} m")
        """
        while True:
            if currently_task == "sensorCalibration":
                conditions = 0
                sensors_data = calibration.get_values()

                print("\n=== SENSOR CALIBRATION DATA ===")
                for key, value in sensors_data.items():
                    if isinstance(value, dict):
                        print(f"{key.upper()}:")
                        for subkey, subvalue in value.items():
                            print(f"   {subkey}: {subvalue}")
                    else:
                        print(f"{key}: {value}")
                print("===============================\n")

                if sensors_data["environment"]["altitude_bme280"] is not None and sensors_data["environment"]["altitude_bme280"] >= min_altitude_air:
                    conditions += 1

                if sensors_data["gps"]["altitude_gps"] is not None and (sensors_data["gps"]["altitude_gps"] - alt_ref) >= min_altitude_air:
                    conditions += 1

                if conditions >= 2:
                    print("Condiciones de lanzamiento detectadas → Cambiando a task: inAir")
                    currently_task = "inAir"

            if currently_task=="inAir":
                #Activar lora
                conditions = 0
                sensors_data=calibration.get_values()
                if sensors_data["environment"]["altitude_bme280"] is not None and sensors_data["environment"]["altitude_bme280"] <= 7:
                    conditions += 1
                if sensors_data["gps"]["altitude_gps"] is not None and (sensors_data["gps"]["altitude_gps"] - alt_ref) <= 7:
                    conditions += 1
                if conditions >= 2:
                    print("Condiciones de lanzamiento detectadas → Cambiando a task: inAir")
                    currently_task = "nicrom"
            if currently_task=="nicrom":
                #Activar nicrom
                
                
                #calibrar sensores:
                sensors_data = calibration.get_values()

                print("\n=== SENSOR CALIBRATION DATA ===")
                for key, value in sensors_data.items():
                    if isinstance(value, dict):
                        print(f"{key.upper()}:")
                        for subkey, subvalue in value.items():
                            print(f"   {subkey}: {subvalue}")
                    else:
                        print(f"{key}: {value}")
                print("===============================\n")
                robot.active_calibration_bno055()

                sensors_data = calibration.get_values()

                print("\n=== SENSOR CALIBRATION DATA ===")
                for key, value in sensors_data.items():
                    if isinstance(value, dict):
                        print(f"{key.upper()}:")
                        for subkey, subvalue in value.items():
                            print(f"   {subkey}: {subvalue}")
                    else:
                        print(f"{key}: {value}")
                print("===============================\n")
                currently_task="GPSControl"
            if currently_task=="GPSControl":
                start = time.time()
                print("epoch: %d" %epoch)
                gps_enabled = (epoch > 0 and (epoch % 100 == 0))
                rover_manager.execute_with_filter(gps_enabled = gps_enabled)
                epoch += 1
                current_point, _ = robot.gps.read()
                distancia = np.linalg.norm(current_point.toENU(target))
                if distancia <= 5:
                   print("Objetivo alcanzado (dentro de 5 metros)")
                   robot.stop()  # O cambia de estado/tarea
                   currently_task=tasks[0]  # O break, según tu estructura
                elapsed = time.time() - start
                time.sleep(max(0, dt - elapsed))
            if currently_task=="CamaraControl":
                pass
            
            """
            print(f"\nEpoch: {epoch}")

            # Activamos GPS cada 100 ciclos
            gps_enabled = (epoch > 0 and (epoch % 100 == 0))

            # Ejecutamos la misión (esta maneja los 5 tasks)
            mission.run(gps_enabled)

            # Lecturas extra de sensores ambientales/energía
            temp, press, hum = bme280.read_data()
            voltage = ina226.read_bus_voltage()
            print(f"[BME280] T={temp:.2f}°C, P={press:.2f}hPa, H={hum:.2f}%")
            print(f"[INA226] Vbat={voltage:.3f} V")

            # Delay para mantener frecuencia de loop
            time.sleep(dt)
            epoch += 1
            """
    except KeyboardInterrupt:
        print("\n🛑 Misión interrumpida por el usuario.")
        robot.update_speed(0, 0)


if __name__ == "__main__":
    main()

