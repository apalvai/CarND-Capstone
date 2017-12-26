#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
	# rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
	
        # TODO: Add other member variables you need below
	self.current_pose = None
	self.base_waypoints = None
	self.closest_waypoint_ahead = None
	self.final_waypoints = Lane()
	self.deceleration = rospy.get_param('~deceleration', 5) # Acceleration should not exceed 10 m/s^2
	self.target_velocity = rospy.get_param('~target_velocity', 22.3)
	params = rospy.get_param_names()
	for i in range(len(params)):
		rospy.logdebug("%s", params[i])

        rospy.spin()

    def pose_cb(self, msg):
        # Save the current_pos
	self.current_pose = msg
	# rospy.logdebug('current position: %f,%f', self.current_pose.pose.position.x, self.current_pose.pose.position.y)	
	
	# Publish the final_waypoints
	if self.final_waypoints is not None:
            self.final_waypoints_pub.publish(self.final_waypoints)

    def waypoints_cb(self, waypoints):
        # Save the received waypoints
	self.base_waypoints = waypoints.waypoints
	# rospy.logdebug('received waypoints: %d', len(self.base_waypoints))

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message. Implement
	light_waypoint_index = msg.data
	if light_waypoint_index is None:
	    return
	
	self.final_waypoints, final_waypoint_indices = self.get_next_waypoints()
	if self.final_waypoints is None or final_waypoint_indices is None:
	   return
	
	if light_waypoint_index == -1:
	    # There is no RED light - drive at target velocity
	    for index in range(len(self.final_waypoints.waypoints)):
		self.set_waypoint_velocity(self.final_waypoints.waypoints, index, self.target_velocity)
	else:
	    # RED light is ON - decelerate to STOP at light_waypoint_index
	    rospy.logdebug('Found a traffic light in RED state')
	    for index, waypoint_index in enumerate(final_waypoint_indices):
		target_velocity = 0.0
		# distance = self.distance(self.base_waypoints, waypoint_index, light_waypoint_index)
		# if distance > 0:
		#     target_velocity = math.sqrt(distance * 2 * self.deceleration)
		
		# chose the minimum velocity
		target_velocity = min(self.target_velocity, target_velocity)
	        self.set_waypoint_velocity(self.final_waypoints.waypoints, index, target_velocity)
	
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    # Helper functions
    def get_next_waypoints(self):
	
	closest_waypoint_index = self.get_closest_waypoint_index_ahead()
	if closest_waypoint_index is None or self.base_waypoints is None:
	    rospy.logdebug('Unable to find closest_waypoint_index_ahead')
	    return
	
	total_waypoints = len(self.base_waypoints)
	
	indices = []
	lane = Lane()
	for i in range(LOOKAHEAD_WPS):
	    index = (closest_waypoint_index + i) % total_waypoints
	    lane.waypoints.append(self.base_waypoints[index])
	    indices.append(index)
	
        return lane, indices

    def get_closest_waypoint_index_ahead(self):
	# Determine the closest_waypoint from current position
	closest_waypoint_index = self.get_closest_waypoint_index()
	if closest_waypoint_index is None or self.base_waypoints is None:
	    rospy.logdebug('Unable to find closest_waypoint_index')
	    return
	
	closest_waypoint = self.base_waypoints[closest_waypoint_index]
	# rospy.logdebug('closest : %s, %s ', closest_waypoint.pose.pose.position.x, closest_waypoint.pose.pose.position.y)
	
	# check whether the closest_waypoint is behind the vehicle or ahead. If behind, chose the waypoint next to it.
	x, y = self.get_local_coordinates(closest_waypoint)
	
	total_waypoints = len(self.base_waypoints)
	
	if x < 0:
	    closest_waypoint_index = (closest_waypoint_index + 1) % total_waypoints
	
	return closest_waypoint_index

    def get_closest_waypoint_index(self):
	# Based on current position and base_waypoints, compute the closest waypoint ahead.
	if self.current_pose is None or self.base_waypoints is None:
	    rospy.logdebug('Missing current_pose or base_waypoints')
	    return
	
	min_distance = 1E7
	closest_waypoint_index = -1E4
	dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
	for index, waypoint in enumerate(self.base_waypoints):
	    distance = dl(self.current_pose.pose.position, waypoint.pose.pose.position)
	    
	    if distance < min_distance:
	        min_distance = distance
		closest_waypoint_index = index
	
	return closest_waypoint_index

    def get_local_coordinates(self, waypoint):
	# Convert given waypoint from global to car's local coordinates.
	
	if self.current_pose is None or waypoint is None:
	    return
	
	cx, cy = self.current_pose.pose.position.x, self.current_pose.pose.position.y
	wx, wy = waypoint.pose.pose.position.x, waypoint.pose.pose.position.y
	
	_,yaw,_ = self.get_angular_params(self.current_pose.pose.orientation)	
	
	# shift
	x, y = wx - cx, wy - cy
	
	# rotate
	local_x = x * math.cos(yaw) + y * math.sin(yaw)
	local_y = y * math.cos(yaw) - x * math.sin(yaw)
	
	return local_x, local_y

    def get_angular_params(self, orientation):
	
	if orientation is None:
	    return
	
	roll, pitch, yaw = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
	
	return roll, pitch, yaw

    # Pre-defined Helper functions
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
