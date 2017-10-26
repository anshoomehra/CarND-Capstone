
from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import rospy
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_SPEED = 30.0


class Controller(object):

	def __init__(self, *args, **kwargs):
		# TODO: Implement
		
		## Initialize Yaw Controller Object .. 
		self.yaw_controller = YawController(kwargs['wheel_base'], kwargs['steer_ratio'],
										 kwargs['min_speed'], kwargs['max_lat_accel'],
										 kwargs['max_steer_angle'])
		
		#self.throttle_pid = PID(kp=0.05, ki=0.015, kd=0.15, mn=kwargs['decel_limit'], mx=kwargs['accel_limit'])
		
		## Compute rate of change for throttle using PID
		self.throttle_pid = PID(kp=0.2, ki=0.0003, kd=3.0, mn=kwargs['decel_limit'], mx=kwargs['accel_limit'])

		## Minimum Speed placeholder 
		self.min_speed = kwargs['min_speed']

		## Placeholders for effective breaking .. 
		self.wheel_base = kwargs['wheel_base']
		self.total_mass = kwargs['vehicle_mass'] # Plus Other compluted mass like fuel
												 # And even how air pressure would effect mass distrubution

		## Time Logging Placeholder 
		self.prev_time = None

		### Apply low pass filter 
		#self.s_lpf = LowPassFilter(tau = .2, ts = .1)
		self.s_lpf = LowPassFilter(tau = 3, ts = 1)

	# control method: for any preprocessing of throttle, brake, yaw
	def control(self, *args, **kwargs):

		# Retrieve present throttle, brake, steer
		target_velocity_linear_x = args[0]
		target_velocity_angular_z = args[1]
		current_velocity_linear_x = args[2]
		current_velocity_angular_z = args[3]
		dbw_enabled = args[4]
		
		throttle = 0.0
		brake = 0.0

		## If DBW is interrupted, reset throttle to avoid errors & return .. 
		if not dbw_enabled:
			self.throttle.reset()
			return 0, 0, 0

		# Compute difference between target and current velocity as CTE for throttle. 
		diff_velocity = target_velocity_linear_x - current_velocity_linear_x

		current_time = rospy.get_time()
		dt = 0
		
		# Compute Delta Time from last compute..
		if self.prev_time is not None: 
			dt = current_time - self.prev_time
		self.prev_time = current_time
		
		velocity_controller = 0
		
		# Determine if throttle or braking is needed .. 
		if dt > 0:
			velocity_controller = self.throttle_pid.step(diff_velocity, dt)
		if velocity_controller > 0:
			throttle = velocity_controller
			# if throttle > 0.7 :  #### Needed for slow machines .. 
			# 	throttle = 0.7
		elif velocity_controller < 0:
			# So braking should be ideaaly dependent in vehcile mass, air pressure ..
			# wheel base etc, we are keeping it simple, as we do not have all the data ..
			# Example : abs(self.total_mass * velocity_controller * self.wheel_base)
			brake = -velocity_controller/2 # reducing braking to half, just my system ..

		# Define yaw from yaw conrtoller, given target and present linear, angular velocities ..
		steering = self.yaw_controller.get_steering(target_velocity_linear_x, target_velocity_angular_z, current_velocity_linear_x)
		steering = self.s_lpf.filt(steering)

		return throttle, brake, steering