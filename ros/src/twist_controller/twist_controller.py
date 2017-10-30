import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.yaw_ctrl = YawController(kwargs['wheel_base'],
                                      kwargs['steer_ratio'],
                                      20, # TODO: decide what is min speed
                                      kwargs['max_lat_accel'],
                                      kwargs['max_steer_angle'])         

        self.pid = PID(kp=5, ki=0.5, kd=0.5, mn=kwargs['decel_limit'], mx=kwargs['accel_limit'])
        self.s_LowPassFilter = LowPassFilter(tau = 3, ts = 1)
        self.t_LowPassFilter = LowPassFilter(tau = 3, ts = 1)

        self.brake_deadband=kwargs['brake_deadband']
        self.vehicle_mass = kwargs['vehicle_mass'] 
        self.fuel_capacity = kwargs['fuel_capacity']
        self.wheel_radius = kwargs['wheel_radius']
        
    def reset(self):
        self.pid.reset()

    def control(self, twist_cmd, current_velocity, del_time):
        
        lin_vel = abs(twist_cmd.twist.linear.x)
        ang_vel = twist_cmd.twist.angular.z
        vel_err = lin_vel - current_velocity.twist.linear.x
        


        next_steer = self.yaw_ctrl.get_steering(lin_vel, ang_vel, current_velocity.twist.linear.x)
        next_steer = self.s_LowPassFilter.filt(next_steer)

        acceleration = self.pid.step(vel_err, del_time)
        acceleration = self.t_LowPassFilter.filt(acceleration)

        if acceleration > 0.0:
            throttle = acceleration
            brake = 0.0
        else:
            throttle = 0.0
            deceleration = -acceleration

            if deceleration < self.brake_deadband:
                deceleration = 0.0

            brake = deceleration * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius

        # Return throttle, brake, steer
        return throttle, brake, next_steer
