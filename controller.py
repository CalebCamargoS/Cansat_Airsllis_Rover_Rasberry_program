import math

class PIDController():
    def __init__(self, robot):
        self.robot = robot
        self.speed = 0.4
        self.kp = 0.1
        self.ki = 0
        self.kd = 0
        self.previous_error = 0
        self.integral_error = 0

    def control_with_gps(self, target):

        current_point,_ = self.robot.gps.read()
        print("posicion actual en formato (lat,long):","(",current_point.latitude,",",current_point.longitude,")")
        target_theta = current_point.bearingTo(target)
        
        u_theta = -target_theta - self.robot.theta
        print("angulo posicion actual -> posicion objetivo",target_theta)
        print("angulo medido por el bno055:",self.robot.theta)
        print("Tetha_objetivo-Tetha_medido con el BNO055",u_theta)

        current_error = math.atan2(math.sin(u_theta), math.cos(u_theta))

        differential_error = current_error - self.previous_error
        integral_error = current_error + self.integral_error

        self.previous_error = current_error
        self.integral_error = integral_error

        w = current_error * self.kp + differential_error * self.kd + integral_error * self.ki

        print('w: %f' %w)

        return self.speed, w


    def control(self, sphericalTarget):
        target = sphericalTarget.toENU(self.robot.reference)

        u_x = target[0] - self.robot.x
        u_y = target[1] - self.robot.y

        target_theta = math.atan2(u_y, u_x)
        current_heading = self.robot.bno055.get_heading_radians()
        current_heading = math.atan2(math.sin(current_heading),math.cos(current_heading))
        #current_heading = self.robot.theta
        print("teta robot:",math.degrees(current_heading))
        print("teta bearing",math.degrees(target_theta))
        u_theta = target_theta - current_heading
        print("delta de teta:",math.degrees(u_theta))

        current_error = math.atan2(math.sin(u_theta), math.cos(u_theta))

        differential_error = current_error - self.previous_error
        integral_error = current_error + self.integral_error

        self.previous_error = current_error
        self.integral_error = integral_error

        w = current_error * self.kp + differential_error * self.kd + integral_error * self.ki

        return self.speed, w

