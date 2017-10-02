from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    #def __init__(self, *args, **kwargs):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 
                self.min_speed, self.max_lat_accel, self.max_steer_angle)

    #def control(self, *args, **kwargs):
    def control(self, linear_velocity, angular_velocity, current_velocity):
        # TODO: Change the arg, kwarg list to suit your needs

        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        # Return throttle, brake, steer

        return 1., 0., steer
