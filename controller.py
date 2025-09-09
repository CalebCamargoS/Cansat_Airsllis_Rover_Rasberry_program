import math
from sphericalTrigonometry import SphericalPoint
import os
class PIDController():
    def __init__(self, robot):
        self.robot = robot
        self.speed = 0.425
        self.kp = 0.8
        self.ki = 0
        self.kd = 0
        self.previous_error = 0
        self.integral_error = 0


    def control(self, target):
        # Leer la última posición GPS desde un archivo txt (formato: lat,lon) generado por el proceso de adquisición de GPS
        
        gps_file = "gps_data.txt"
        lat, lon = None, None
        if os.path.exists(gps_file):
            with open(gps_file, 'r') as f:
                line = f.readline().strip()
                try:
                    lat, lon = map(float, line.split(','))
                except Exception:
                    lat, lon = None, None
        if lat is None or lon is None:
            # Si no hay datos, mantener el control anterior (o puedes retornar 0,0)
            return self.speed, 0.0

        
        current_point = SphericalPoint(lat, lon)
        target_theta = current_point.bearingTo(target)
        target_theta = math.atan2(math.sin(target_theta), math.cos(target_theta))
        current_heading = self.robot.bno055.get_heading_radians()
        current_heading = math.atan2(math.sin(current_heading), math.cos(current_heading))
        u_theta = target_theta - current_heading
        current_error = math.atan2(math.sin(u_theta), math.cos(u_theta))
        differential_error = current_error - self.previous_error
        integral_error = current_error + self.integral_error

        self.previous_error = current_error
        self.integral_error = integral_error

        w = current_error * self.kp + differential_error * self.kd + integral_error * self.ki

        return self.speed, w

