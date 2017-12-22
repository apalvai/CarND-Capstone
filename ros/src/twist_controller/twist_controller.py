
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, yaw_controller):
        # TODO: Implement
	self.yaw_controller = yaw_controller
	
	kp = 0.2
	ki = 0.001
	kd = 0.1
	min_val = -1.0
	max_val = 1.0

	self.pid = PID(kp, ki, kd, mn=min_val, mx=max_val)

    def control(self, desired_linear_velocity, desired_angular_velocity, current_linear_velocity, sample_time):
        
	# Determine the throttle/brake based on desired and current velocities using PID
	error = desired_linear_velocity - current_linear_velocity
	step_val = self.pid.step(error, sample_time)
	
	if step_val > 0.0:
	    throttle = step_val
	else:
	    brake = -step_val
	
	# Determine steering angle based on desired and current velocities using YawController
	steer = self.yaw_controller.get_steering(desired_linear_velocity, desired_angular_velocity, current_linear_velocity)
	
	# Return throttle, brake, steer
        return throttle, brake, steer
