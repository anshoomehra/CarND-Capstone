#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
from itertools import islice, cycle
import copy

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
DEBUG = False
MAX_ACCL = 20./2.23693
MAX_DECEL = 1.

class WaypointUpdater(object):
	def __init__(self):
		rospy.init_node('waypoint_updater')

	# Subscribers
		# Provides base waypoints provided by Udacity Simulator, published only once..  
		rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
		# Provides current pose statistics of the car..
		rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
		# Provides waypoint of ahead traffic light with red signal ..
		rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

		rospy.Subscriber('/current_velocity', TwistStamped, 
						  self.current_velocity_cb)
		
		# TODO: Add a subscriber for /obstacle_waypoint below

	# Publishers
		# Publish computed final waypoints 
		self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

		# TL Placholders
		self.tl_red_waypoint_idx = -1
		
		# Waypoints Placeholders
		self.base_waypoints = None
		self.number_of_base_waypoints = None

		# Pose Placeholders
		self.current_pose = None
		self.car_x = None
		self.car_y = None
		self.min_dist_from_car_idx = None

		# Velocity Placeholders
		self.current_velocity = None
		self.max_velocity = 10./2.23693 # m/s
		self.current_velocity = None
		
		# Loop until interrupt is issued as closing simulator or Ctrl+C as examples
		rospy.spin()

	# Compute velocity, and set each waypoint with target velocity .. 
	def update_waypoints_velocity(self, waypoints):
		for i in range(len(waypoints)):
			waypoints[i].twist.twist.linear.x = self.max_velocity

		return waypoints

	def accelerate(self, min_dist_from_car_idx, lane):
		for idx in range(min_dist_from_car_idx, min_dist_from_car_idx+LOOKAHEAD_WPS):
			# Ensure index is not out range and is cyclic
			idx = idx % self.number_of_base_waypoints
			# Update the target velocity along ..
			self.set_waypoint_velocity(self.base_waypoints, idx , MAX_ACCL)
			# Add to the list of final waypoints 
			lane.waypoints.append(self.base_waypoints[idx])
		return lane

	def euclidean_distance_2d(self, position1, position2):
		a = position1
		b = position2
		return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

	def decelerate(self, min_dist_from_car_idx, lane):
		## Document
		'''
		Experimental Ground
		'''

		distToNextWP = self.euclidean_distance_2d(self.current_pose.position, self.base_waypoints[min_dist_from_car_idx].pose.pose.position)
		stopDist = distToNextWP + self.distance(self.base_waypoints, min_dist_from_car_idx, self.tl_red_waypoint_idx)
		fullStopVelocity = math.sqrt(2 * MAX_DECEL * stopDist)

		#Target Velocity
		v0 = min(fullStopVelocity, self.base_waypoints[min_dist_from_car_idx].twist.twist.linear.x)

		for i in range (min_dist_from_car_idx, min_dist_from_car_idx+LOOKAHEAD_WPS):
			newWpVelocity = v0 * max(0., stopDist - distToNextWP) / stopDist
			
			if newWpVelocity < 0.1:
				newWpVelocity = 0.

			wp = copy.deepcopy(self.base_waypoints[i])
			wp.twist.twist.linear.x  = newWpVelocity

			lane.waypoints.append(wp)

			if (i < self.tl_red_waypoint_idx):
				distToNextWP += self.distance(self.base_waypoints, i, i+1)

		return lane
		'''
		for idx in range(min_dist_from_car_idx, self.tl_red_waypoint_idx):
			# Initial, Target Velocity & Distance to stop point
			initial_velocity =  self.current_velocity.linear.x
			target_velocity = 0.
			distance_to_stop_line = self.distance(self.base_waypoints, idx, self.tl_red_waypoint_idx)

			# 	# Deceleration Formula
			# 	# -*- coding: utf-8 -*-
			# 	# -a = ( v - u ) / t  :: v: Final Velocity, u: Initial Velocity, t: time to decelerate
			# 	# -a = (v**2 -u**2) / 2s  :: v: Final Velocity, u: Initial Velocity, s: ditance to travel
			new_acceleration = round(( (target_velocity)**2 - (initial_velocity)**2 ) / 2 * distance_to_stop_line, 2) 

			#rospy.loginfo("**Debug Idx, MinCarIdx, TLIdx, DistToStop, IdxVel, NewAcc: {}, {}, {}, {}, {}, {}".format(idx, min_dist_from_car_idx, self.tl_red_waypoint_idx, distance_to_stop_line, initial_velocity, new_acceleration ))
			# if new_acceleration < 0:
			# 	new_acceleration = 0

			# Ensure index is not out range and is cyclic
			idx = idx % self.number_of_base_waypoints
			# Update the target velocity along ..
			self.set_waypoint_velocity(self.base_waypoints, idx , new_acceleration)
			# Add to the list of final waypoints 
			lane.waypoints.append(self.base_waypoints[idx])
		
		return lane
		'''

	# Helper Method:
	# Compute Final Waypoints, and publish them to /final_waypoints node
	def send_final_waypoints(self):
		# Car's present position
		car_x = self.current_pose.position.x
		car_y = self.current_pose.position.y

		# Minimum Waypoint Distance Placeholders
		self.min_dist_from_car = 99999

		# Local Placeholders 
		wp_start_idx = 0 # Index to compute waypoints from base list
		wp_end_idx = self.number_of_base_waypoints # Index until computing waypoints from base list 

		# If this is true, it means it is not the first time this event has triggered
		# set the start and end index according to where car is, 20 before and either 
		# 20 after or number of total waypoints whichever is less .. 
		# instead of looping all waypoints to gain performance .. 
		if (self.min_dist_from_car_idx is not None):
			wp_start_idx = self.min_dist_from_car_idx - 100  #[ remove minus ]
			wp_end_idx = min( self.number_of_base_waypoints , self.min_dist_from_car_idx+200 )

		# Find minimum distance car and it's index
		for idx in range(wp_start_idx, wp_end_idx):
			waypoint = self.base_waypoints[idx]
			wp_x = waypoint.pose.pose.position.x
			wp_y = waypoint.pose.pose.position.y

			# Compute Distance : sqrt ( (x1-x2)^2 , (y1-y2)^2 )
			distance = math.sqrt( (car_x - wp_x)**2 + (car_y - wp_y)**2 )

			# Register minimum distance
			if distance < self.min_dist_from_car:
				self.min_dist_from_car = distance
				self.min_dist_from_car_idx = idx

		# Closest waypoint position ... For Logging ..
		#closest_wp_pos = self.base_waypoints[self.min_dist_from_car_idx].pose.pose.position

		lane = Lane()

		# Filter the waypoints which are ahead of the car
		# Caution: Since it is loop, waypoints are cyclic,
		#          ensure loop s index back ..
		#rospy.loginfo("red Light waypoint index: {}".format(self.tl_red_waypoint_idx))

		#if self.tl_red_waypoint_idx is None or self.tl_red_waypoint_idx < 0 or (self.min_dist_from_car_idx % self.number_of_base_waypoints) > self.tl_red_waypoint_idx:

		#rospy.loginfo ("Idx, TLIdx, NWP, Cond, DTL: {}, {}, {}, {}, {}".format(self.min_dist_from_car_idx, self.tl_red_waypoint_idx, self.number_of_base_waypoints, ((self.min_dist_from_car_idx % self.number_of_base_waypoints) > self.tl_red_waypoint_idx), (self.tl_red_waypoint_idx-self.min_dist_from_car_idx)))
		
		#if self.tl_red_waypoint_idx is None or self.tl_red_waypoint_idx < 0 or self.min_dist_from_car_idx < (self.tl_red_waypoint_idx - 100):
			## Document acceleration logic ..

		'''
		Experimentation Ground .. 
		'''

		if self.tl_red_waypoint_idx is None or self.tl_red_waypoint_idx < 0:
			rospy.loginfo("Accelerate")
			lane = self.accelerate(self.min_dist_from_car_idx, lane)
		else:
			if self.min_dist_from_car_idx >= self.tl_red_waypoint_idx:
				rospy.loginfo("Accelerate")
				lane = self.accelerate(self.min_dist_from_car_idx, lane)
			else:
				distance_to_tl = self.tl_red_waypoint_idx-self.min_dist_from_car_idx
				if distance_to_tl > 0 and distance_to_tl < 200:
					rospy.loginfo("Decelerate")
					#lane = self.decelerate(self.min_dist_from_car_idx, lane)
				else:
					rospy.loginfo("Accelerate")
					lane = self.accelerate(self.min_dist_from_car_idx, lane)
		

		#lane = self.accelerate(self.min_dist_from_car_idx, lane)

		# for idx in range(self.min_dist_from_car_idx, self.min_dist_from_car_idx+LOOKAHEAD_WPS):
		# 	# Ensure index is not out range and is cyclic
		# 	idx = idx % self.number_of_base_waypoints
		# 	# Update the target velocity along ..
		# 	self.set_waypoint_velocity(self.base_waypoints, idx , 20./2.23693)
		# 	# Add to the list of final waypoints 
		# 	lane.waypoints.append(self.base_waypoints[idx])
		# #else:
			## Document deceleration logic
			

		# else: # We have stop light ahead, set the gradual decrease in velocity ..
		# 	# Deceleration Formula
		# 	# -*- coding: utf-8 -*-
		# 	# -a = ( v - u ) / t  :: v: Final Velocity, u: Initial Velocity, t: time to decelerate
		# 	# -a = (v**2 -u**2) / 2s  :: v: Final Velocity, u: Initial Velocity, s: ditance to travel

		# 	for idx in range(self.min_dist_from_car_idx, self.tl_red_waypoint_idx):

		# 		# Initial, Target Velocity & Distance to stop point
		# 		initial_velocity =  self.current_velocity.linear.x #self.get_waypoint_velocity(self.base_waypoints[idx]) ### This is coming zero ..
		# 		#initial_velocity = self.current_pose.
		# 		target_velocity = 0.
		# 		distance_to_stop_line = self.distance(self.base_waypoints, idx, self.tl_red_waypoint_idx)

		# 		new_acceleration = ( (target_velocity)**2 - (initial_velocity)**2 ) / 2 * distance_to_stop_line

		# 		rospy.loginfo("**Debug Idx, MinCarIdx, TLIdx, DistToStop, IdxVel, NewAcc: {}, {}, {}, {}, {}, {}".format(idx, self.min_dist_from_car_idx, self.tl_red_waypoint_idx, distance_to_stop_line, initial_velocity, new_acceleration ))
		# 		if new_acceleration < 0:
		# 			new_acceleration = 0

		# 		# Ensure index is not out range and is cyclic
		# 		idx = idx % self.number_of_base_waypoints
		# 		# Update the target velocity along ..
		# 		self.set_waypoint_velocity(self.base_waypoints, idx , new_acceleration)
		# 		# Add to the list of final waypoints 
		# 		lane.waypoints.append(self.base_waypoints[idx])
		# #Waypoints, and velocities are set, time to Publish waypoints 
		#to /final_waypoints node ..
		if DEBUG :
			rospy.loginfo("Publishing next waypoints to final_waypoints")

		self.final_waypoints_pub.publish(lane)

	# Call Back Method for /traffic_waypoint:
	# Retrieve TL Red Waypoints..
	def traffic_cb(self, msg):
		#rospy.loginfo("In Traffic CB")
		self.tl_red_waypoint_idx = msg.data        
		#rospy.loginfo("Detected light: " + str(msg.data))
		if self.base_waypoints is not None: #self.tl_red_waypoint_idx > -1:
			self.send_final_waypoints()
			
	# Call Back Method for /current_pose:
	# Compute Final Waypoints, and publish them to /final_waypoints node
	def pose_cb(self, PoseStampedMsg):
		
		if DEBUG :
			rospy.loginfo("In Pose CB...")
		# TODO: Implement

		self.current_pose = PoseStampedMsg.pose
		#Log Message Later ...
		if DEBUG :
			rospy.loginfo("Current Pose {} , {}, {}".format(self.current_pose.position.x,
															self.current_pose.position.y, 
															self.current_pose.position.z))

		# Publish final waypoints ..
		## Wait until base_waypoits are published, call backs are not in sequence &
		## are unpredictable, need to assert required variables prior processing .. 
		if self.base_waypoints is not None: 
			self.send_final_waypoints()
 
	# Call Back Method for /base_waypoints:
	# Compute Final Waypoints, and publish them to /final_waypoints node
	def waypoints_cb(self, LaneMsg):

		if DEBUG :
			rospy.loginfo("In Waypoints CB...")

		# Log Message Later ...
		#print ("Current Velocity:", waypoints.twist.twist.linear.x, waypoints.twist.twist.linear.y, waypoints.twist.twist.linear.z)
		self.base_waypoints = LaneMsg.waypoints
		self.number_of_base_waypoints = len(LaneMsg.waypoints)

		if DEBUG :
			rospy.loginfo("In Waypoints CB, total number of waypoints {}...".format(self.number_of_base_waypoints))

		# Publish final waypoints ..
		## Wait until base_waypoits are published, call backs are not in sequence &
		## are unpredictable, need to assert required variables prior processing .. 
		if self.current_pose is not None:
			self.send_final_waypoints()

	def obstacle_cb(self, msg):
		# TODO: Callback for /obstacle_waypoint message. We will implement it later
		pass

	## Call back funtion for /current_velocity
	def current_velocity_cb(self, TwistStampedMsg):
		self.current_velocity = TwistStampedMsg.twist

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

# MAIN : Call WaypointUpdater(), which will run indefinite until interrupt is
# trigerred ..
if __name__ == '__main__':
	try:
		WaypointUpdater()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start waypoint updater node.')
