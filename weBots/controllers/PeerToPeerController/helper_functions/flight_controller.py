from controller import Robot

from math import cos, sin
from time import sleep

from helper_functions.pid_controller import pid_velocity_fixed_height_controller
from helper_functions.path_controller_class import PathController
from helper_functions.drone_state import DroneState
from helper_functions.radio_controller import radio_controller
from helper_functions.peer_to_peer_class import PeerToPeerClass

FLYING_ATTITUDE = 1
# communication mode
# eithre peer_to_peer or crazyradio
COMMUNICATION_MODE = "peer_to_peer"
# COMMUNICATION_MODE = "crazyradio"

class FlightController():
	def __init__(self):

		self.robot = Robot()
		self.radio = radio_controller(self.robot)
		self.name = self.robot.getName()
		self.peertopeer = PeerToPeerClass(self.name)
		self.droneState = DroneState.INACTIVE
		self.timestep = int(self.robot.getBasicTimeStep())

		# Initialize motors
		self.m1_motor = self.robot.getDevice("m1_motor")
		self.m1_motor.setPosition(float('inf'))
		self.m1_motor.setVelocity(-1)
		self.m2_motor = self.robot.getDevice("m2_motor")
		self.m2_motor.setPosition(float('inf'))
		self.m2_motor.setVelocity(1)
		self.m3_motor = self.robot.getDevice("m3_motor")
		self.m3_motor.setPosition(float('inf'))
		self.m3_motor.setVelocity(-1)
		self.m4_motor = self.robot.getDevice("m4_motor")
		self.m4_motor.setPosition(float('inf'))
		self.m4_motor.setVelocity(1)

		# Initialize Sensors
		self.imu = self.robot.getDevice("inertial_unit")
		self.imu.enable(self.timestep)
		self.gps = self.robot.getDevice("gps")
		self.gps.enable(self.timestep)
		self.gyro = self.robot.getDevice("gyro")
		self.gyro.enable(self.timestep)
		self.camera = self.robot.getDevice("camera")
		self.camera.enable(self.timestep)

		# initialize radio controller
		self.receiver = self.robot.getDevice("receiver")
		self.receiver.enable(self.timestep)
		self.emitter = self.robot.getDevice("emitter")


		# Initialize variables
		self.past_x_global = 0
		self.past_y_global = 0
		self.past_time = 0
		self.first_time = True

		# Path
		self.trajectory_index = 0

		if self.name == "Crazyflie1":
			self.waypoints = [[0, 0, 1]]
		if self.name == "Crazyflie2":
			self.waypoints = [[0, 0, 1]]
		if self.name == "Crazyflie3":
			self.waypoints = [[0, 0, 1]]

		# crazyflie trajectories for swarm (peer to peer)
		self.crazyflie1_trajectory = []
		self.crazyflie2_trajectory = []
		self.crazyflie3_trajectory = []

		# Crazyflie velocity PID controller
		self.PID_crazyflie = pid_velocity_fixed_height_controller()
		self.PID_update_last_time = self.robot.getTime()
		self.sensor_read_last_time = self.robot.getTime()

		print("Initialising: ", self.name)

	# Main loop
	def run(self):
		# Initialize values
		forward_desired = 0
		sideways_desired = 0
		yaw_desired = 0
		height_desired = FLYING_ATTITUDE

		while self.robot.step(self.timestep) != -1:

			dt = self.robot.getTime() - self.past_time

			if self.first_time:
					self.past_x_global = self.gps.getValues()[0]
					self.past_y_global = self.gps.getValues()[1]
					self.past_time = self.robot.getTime()
					self.first_time = False
					self.droneState = DroneState.TAKEOFF
					path_controller = PathController(self.robot, self.gps, self.imu, self.gyro, self.timestep, self.waypoints[self.trajectory_index])
					# add current position infront of the trajectory
					self.waypoints.insert(0, [self.gps.getValues()[0], self.gps.getValues()[1], FLYING_ATTITUDE])

			# Get sensor data
			roll = self.imu.getRollPitchYaw()[0]
			pitch = self.imu.getRollPitchYaw()[1]
			yaw = self.imu.getRollPitchYaw()[2]
			yaw_rate = self.gyro.getValues()[2]
			x_global = self.gps.getValues()[0]
			v_x_global = (x_global - self.past_x_global)/dt
			y_global = self.gps.getValues()[1]
			v_y_global = (y_global - self.past_y_global)/dt
			altitude = self.gps.getValues()[2]

			# Get body fixed velocities
			cos_yaw = cos(yaw)
			sin_yaw = sin(yaw)
			v_x = v_x_global * cos_yaw + v_y_global * sin_yaw
			v_y = - v_x_global * sin_yaw + v_y_global * cos_yaw

			# if drone crashes, change state to INACTIVE (upside down, or collision with ground)
			if pitch > 0.5 or pitch < -0.5 or roll > 0.5 or roll < -0.5 or yaw_rate > 2 or yaw_rate < -2:
				print("Crash")
				self.droneState = DroneState.INACTIVE

			# check if a new packet is received
			#if self.receiver.getQueueLength() > 0:
			#	self.radio.receive_packet()

			# every half second, send a packet with the drone's position and desired trajectory
			if self.robot.getTime() - self.sensor_read_last_time > 0.5:
				position_packet = [x_global, y_global, altitude]
				self.radio.send_packet(self.name, "POS", position_packet)
				# trajectory_packet = self.waypoints
				# self.radio.send_packet(self.name, "TRAJ", trajectory_packet)

				self.sensor_read_last_time = self.robot.getTime()

			# Handle Takeoff state
			if self.droneState == DroneState.TAKEOFF:
				# send current trajectory to other drones/crazyradio
				# depending on communication mode, send the packet to the crazyradio for computation or do it self (peer to peer)
				# add current position to the trajectory
				# send the trajectory to the other drones
				# if the drone receives a trajectory, change state to FLYING
				# if the drone doesn't receive a trajectory, keep sending the current trajectory
				self.radio.send_packet(self.name, "TRAJ", self.waypoints)
				is_safe_to_fly = False
				
				if COMMUNICATION_MODE == "peer_to_peer":
					# when barrier function returns true, and a new trajcetory, start flying
					# add own trajectory
					self.peertopeer.add_trajectory(self.name, self.waypoints)
					if self.receiver.getQueueLength() > 0:
						packet = self.radio.receive_packet()
						# if not it's own trajectory, use peertopeer class to compute the barrier function
						if packet[0] != self.name and packet[1] == "TRAJ":
							self.peertopeer.add_trajectory(packet[0], packet[2])
						# if all trajectories are received, compute the new trajectory
						if self.peertopeer.has_all_trajectories():
							print("All trajectories received")
							# compute new trajectory
							self.peertopeer.run()
							self.waypoints = self.peertopeer.get_trajectory()
							is_safe_to_fly = True
				if COMMUNICATION_MODE == "crazyradio":
					# when barrier function returns true, and a new trajcetory, start flying
					if self.receiver.getQueueLength() > 0:
						packet = self.radio.receive_packet()
						if packet[0] == self.name and packet[1] == "TRAJ":
							self.waypoints = packet[2]
							is_safe_to_fly = True
					else:
						sleep(0.1)
				if is_safe_to_fly:
					print("Flying")
					self.droneState = DroneState.FLYING



			# Handle Flying state
			if self.droneState == DroneState.FLYING:
				path_controller.set_endpoint(self.waypoints[self.trajectory_index])
				# Path planning
				forward_desired, sideways_desired, yaw_desired, height_desired = path_controller.compute_desired_values()
				if path_controller.check_waypoint_treshold_reached():
					print("Waypoint reached")
					self.trajectory_index += 1
					if self.trajectory_index == len(self.waypoints):
						self.droneState = DroneState.HOVERING

			# Handle Hovering state
				# how to got from hovering to flying again?
			if self.droneState == DroneState.HOVERING:
				forward_desired = 0
				sideways_desired = 0
				yaw_desired = 0
				height_desired = FLYING_ATTITUDE
				# if turning is needed, change state to TURNING
				current_yaw = path_controller.compute_desired_yaw()
				if self.trajectory_index == len(self.waypoints):
					self.droneState = DroneState.LANDING
				else:
					self.droneState = DroneState.FLYING

			# Handle Landing state
			if self.droneState == DroneState.LANDING:
				# slowly descend to the ground, by decreasing the motor power
				current_motor_power = motor_power
				timestamp = self.robot.getTime()
				# decrease motor power
				if timestamp - self.PID_update_last_time > 0.2:
					height_desired -= 0.05
					self.PID_update_last_time = timestamp
				# if drone is close to the ground, change state to INACTIVE
				if altitude < 0.1:
					self.droneState = DroneState.INACTIVE

		
			# Handle Inactive state
			if self.droneState == DroneState.INACTIVE:
				motor_power = [0, 0, 0, 0]
				self.m1_motor.setVelocity(-motor_power[0])
				self.m2_motor.setVelocity(motor_power[1])
				self.m3_motor.setVelocity(-motor_power[2])
				self.m4_motor.setVelocity(motor_power[3])
				self.radio.send_packet(self.name, "NACK", "Drone is inactive")
				break
	
			## Barrier function controller

			# Update the trajectory

			# PID velocity controller with fixed height
			motor_power = self.PID_crazyflie.pid(dt, forward_desired, sideways_desired,
																			yaw_desired, height_desired,
																			roll, pitch, yaw_rate,
																			altitude, v_x, v_y)

			self.m1_motor.setVelocity(-motor_power[0])
			self.m2_motor.setVelocity(motor_power[1])
			self.m3_motor.setVelocity(-motor_power[2])
			self.m4_motor.setVelocity(motor_power[3])

			self.past_time = self.robot.getTime()
			self.past_x_global = x_global
			self.past_y_global = y_global