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
        # Usar la última posición GPS disponible en memoria (no bloqueante)
        current_point = getattr(self.robot.gps, 'last_point', None)
        if current_point is None or (getattr(current_point, 'latitude', 0.0) == 0.0 and getattr(current_point, 'longitude', 0.0) == 0.0):
            # Si no hay datos válidos, mantener el control anterior (o puedes retornar 0,0)
            return self.speed, 0.0

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

