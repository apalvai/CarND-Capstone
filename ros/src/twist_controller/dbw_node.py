#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller
from yaw_controller import YawController

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node', log_level=rospy.DEBUG)

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

	self.current_velocity = None
	self.dbw_enabled = False
	self.twist_cmd = None

	self.rate = 50 # 50Hz
	self.sample_time = 1./self.rate
	
	# Create YawController object
	self.yaw_controller = YawController(wheel_base, steer_ratio, 0.0, max_lat_accel, max_steer_angle)

        # TODO: Create `TwistController` object
        self.controller = Controller(self.yaw_controller)

        # TODO: Subscribe to all the topics you need to
	rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)
	rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)
	rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1)
	
        self.loop()

    def loop(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
	    
	    if self.twist_cmd is not None and self.current_velocity is not None:
	        
	        # Get predicted throttle, brake, and steering using `twist_controller`
            	throttle, brake, steering = self.controller.control(self.twist_cmd.twist.linear.x,
                                                                    self.twist_cmd.twist.angular.z,
                                                                    self.current_velocity.twist.linear.x,
								    self.dbw_enabled,
								    self.sample_time)
		
		rospy.logdebug('throttle: %f, brake: %f, steering: %f', throttle, brake, steering)
	    	
	    	# TODO: You should only publish the control commands if dbw is enabled
            	# if self.dbw_enabled is True:
		rospy.logdebug('Publishing throttle, break and steer commands...')
                self.publish(throttle, brake, steering)
	    
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def current_velocity_cb(self, current_velocity):
	rospy.logdebug('received current velocity: %f, %f, %f', 
			current_velocity.twist.linear.x, 
			current_velocity.twist.linear.y, 
			current_velocity.twist.linear.z)
	self.current_velocity = current_velocity

    def dbw_enabled_cb(self, dbw_enabled):
	rospy.logdebug('dbw_enabled: %r', dbw_enabled)
	self.dbw_enabled = dbw_enabled

    def twist_cmd_cb(self, twist_cmd):
	rospy.logdebug('received twist_cmd: %f, %f, %f', 
			twist_cmd.twist.linear.x, 
			twist_cmd.twist.linear.y, 
			twist_cmd.twist.linear.z)
	self.twist_cmd = twist_cmd

if __name__ == '__main__':
    try:
        DBWNode()
    except rospy.ROSInterruptException:
	rospy.logerr('Failed to initialize DBW node.')
