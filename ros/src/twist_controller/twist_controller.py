import rospy

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
    # def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, accel_limit, decel_limit, vehicle_mass, wheel_radius, brake_deadband):
        # TODO: Implement
        self.wheel_base = args[0]
        self.steer_ratio = args[1]
        self.min_speed = args[2]
        self.max_lat_accel = args[3]
        self.max_steer_angle = args[4]
        self.accel_limit = args[5]
        self.decel_limit = args[6]
        self.vehicle_mass = args[7]
        self.wheel_radius = args[8]
        self.brake_deadband = args[9]

        self.sample_time = 0.02  # 50Hz

        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)
        kp = 1.5    # to counter-steer deviations
        ki = 0.004  # to correct system errors, set to low as we are in a simulator
        kd = 0.1    # to smooth out at large deviations
        self.pid = PID(kp, ki, kd, mn=self.decel_limit, mx=self.accel_limit)
        self.lowpass = LowPassFilter(tau=4, ts=1.) # new_val = 1/5*val + 4/5*last_val

    def control(self, *args, **kwargs):
    # def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        linear_velocity = args[0]
        angular_velocity = args[1]
        current_velocity = args[2]
        dbw_enabled = args[3]

        if dbw_enabled:
            # Return throttle, brake, steer
            steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

            # Apply PID control to throttle
            error = linear_velocity - current_velocity
            throttle = self.pid.step(error, self.sample_time)

            # Apply low pass filter to throttle
            throttle = self.lowpass.filt(throttle)

            # throttle and brake control
            decel = 0.0

            if linear_velocity < 0.1:
                throttle = 0.0
                decel = abs(self.decel_limit)
            else:
                if throttle > 0.0:
                    decel = 0.0
                else:
                    decel = - throttle
                    throttle = 0.0

                    if decel < self.brake_deadband:
                        decel = 0.0

            brake = decel * self.vehicle_mass * self.wheel_radius
            return throttle, brake, steer

        else:
            self.pid.reset()
            return 0.0, 0.0, 0.0



    # reset pid if needed
    def reset(self):
        self.pid.reset()

