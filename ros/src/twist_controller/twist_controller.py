from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    # def __init__(self, *args, **kwargs):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, accel_limit, decel_limit):
        # TODO: Implement
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        self.sample_time = 0.02  # 50Hz

        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)
        kp = 1.5  # to counter-steer deviations
        ki = 0.004  # to correct system errors, set to low as we are in a simulator
        kd = 0.1  # to smooth out at large deviations
        self.pid = PID(kp, ki, kd, mn=decel_limit, mx=accel_limit)
        self.lowpass = LowPassFilter(tau=4, ts=1.) # new_val = 1/5*val + 4/5*last_val

    # def control(self, *args, **kwargs):
    def control(self, linear_velocity, angular_velocity, current_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        # Apply PID control to throttle
        error = linear_velocity - current_velocity
        throttle = self.pid.step(error, self.sample_time)

        # Apply low pass filter to throttle
        throttle = self.lowpass.filt(throttle)

        if linear_velocity > 0.0:
            throttle = throttle
            brake = 0.0

        else:
            throttle = 0.0
            brake = - throttle

        return throttle, brake, steer

    # reset pid if needed
    def reset(self):
        self.pid.reset()
