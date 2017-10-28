from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']

        self.law_pass_filter = LowPassFilter(10,1) # TODO: decide on LPF filter params
        self.yaw_ctrl = YawController(kwargs['wheel_base'],
                                      kwargs['steer_ratio'],
                                      20, # TODO: decide what is min speed
                                      kwargs['max_lat_accel'],
                                      kwargs['max_steer_angle'])

    def control(self, *args, **kwargs):

        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
