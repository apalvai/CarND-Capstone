#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import tf

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
		
	# List of positions that correspond to the line to stop in front of for a given intersection
        self.stop_line_positions = self.config['stop_line_positions']
		
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
		
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
	
	self.use_lights_state = True # Make it false to enable the classifier.

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # Based on current position and waypoints, compute the closest waypoint ahead.
	if pose is None or self.waypoints is None:
	    return
		
	min_distance = 1E7
	closest_waypoint_index = -1E4
	dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
	for index, waypoint in enumerate(self.waypoints):
	    distance = dl(pose.position, waypoint.pose.pose.position)
	    
	    if distance < min_distance:
	        min_distance = distance
		closest_waypoint_index = index
	
	return closest_waypoint_index

    def get_local_coordinates(self, pose_1, pose_2):
	# Convert given waypoint from global to pose_1's local coordinates.
	
	if pose_1 is None or pose_2 is None:
	    return
	
	cx, cy = pose_1.position.x, pose_1.position.y
	wx, wy = pose_2.position.x, pose_2.position.y
	
	_,yaw,_ = self.get_angular_params(pose_1.orientation)	
	
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

    def get_light_state(self):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return TrafficLight.RED

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        
	light_wp = -1
	state = TrafficLight.UNKNOWN
	
	if self.pose is None:
	    return light_wp, state
	
	if self.lights is None:
	    return light_wp, state
	    
	# Find the closest traffic light ahead which is within 100m of the car's current pose
	light = None
	min_dist_x = 75
	for traffic_light in self.lights:
	    dist_x, dist_y = self.get_local_coordinates(self.pose.pose, traffic_light.pose.pose)
	    
	    if dist_x < min_dist_x and dist_x > 0:
	        min_dist_x = dist_x
		light = traffic_light
		
		if self.use_lights_state is True:
	    	    state = light.state
		else:
	    	    state = self.get_light_state()
	
	# Find the stop line (ahead of car) that is closest to the light
	min_distance = 50
	closest_stop_line = None
	if light is not None and self.stop_line_positions is not None:
	    
	    dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
	    for coord in self.stop_line_positions:
		stop_line_pose = Pose()
		stop_line_pose.position.x = coord[0]
		stop_line_pose.position.y = coord[1]
		
		# distance between stop_line and closest light
		distance = dl(stop_line_pose.position, light.pose.pose.position)
		
		# relative position of stop line w.r.t car
		dist_x, dist_y = self.get_local_coordinates(self.pose.pose, stop_line_pose)
		
		if distance < min_distance and dist_x > 0:
		    min_distance = distance
		    closest_stop_line = stop_line_pose
	    
	    # Find the waypoint index that is closest to the stop line
	    if closest_stop_line is not None:
		light_wp = self.get_closest_waypoint(closest_stop_line)

        return light_wp, state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
