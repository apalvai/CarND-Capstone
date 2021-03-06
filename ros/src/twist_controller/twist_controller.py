
import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, yaw_controller, vehicle_mass, wheel_radius):
        # TODO: Implement
	self.yaw_controller = yaw_controller
	self.vehicle_mass = vehicle_mass
	self.wheel_radius = wheel_radius
	
	kp = 0.2
	ki = 0.001
	kd = 0.1
	min_val = -1.0
	max_val = 1.0
	tau = 20.0
	ts = 1.0

	self.pid = PID(kp, ki, kd, mn=min_val, mx=max_val)
	self.lpf = LowPassFilter(tau, ts)

    def control(self, desired_linear_velocity, desired_angular_velocity, current_linear_velocity, dbw_enabled, sample_time):
        
	# rospy.logdebug('desired velocity: %f, desired angular velocity: %f, current velocity: %f', desired_linear_velocity, desired_angular_velocity, current_linear_velocity)
	
	# Reset the pid controller when DBW mode is disabled
	if dbw_enabled is False:
	    self.pid.reset()
	    return 0, 0, 0

	# Determine the throttle/brake based on desired and current velocities using PID
	error = desired_linear_velocity - current_linear_velocity
	step_val = self.pid.step(error, sample_time)
	step_val = self.lpf.filt(step_val)	
	
	throttle = 0
	brake = 0
	
	if desired_linear_velocity < 0.2 and current_linear_velocity < 0.2:
	    brake = self.vehicle_mass * self.wheel_radius
	else:
	    if step_val > 0.0:
	    	throttle = step_val
	    else:
	    	brake = -step_val * self.vehicle_mass * self.wheel_radius * 5
	
	# Determine steering angle based on desired and current velocities using YawController
	steer = self.yaw_controller.get_steering(desired_linear_velocity, desired_angular_velocity, current_linear_velocity)
	
	# Return throttle, brake, steer
        return throttle, brake, steer
