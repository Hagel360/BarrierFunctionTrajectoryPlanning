"""
This module is responsible for following the waypoints given by the user.

The waypoints are given in the form of a list of coordinates. The drone will
follow these coordinates in sequence. The drone will fly to the first
coordinate, then the second, and so on.

The module returns desired forward velocity, desired sideways velocity, desired
yaw rate, desired altitude, and the motor power. These values are used by the
Crazyflie PID motor control to control the drone. 
"""

import numpy as np
from math import cos, sin, sqrt, atan2


SPEED_MULTIPLIER = 0.2

class PathController():
    def __init__ (self, robot, gps, imu, gyro, timestep, endpoint):
        self.endpoint = endpoint
        self.waypoint_tolerance = 0.3
        self.waypoints_reached = False
        self.robot = robot
        self.gps = gps
        self.imu = imu
        self.gyro = gyro
        self.timestep = timestep

    def set_endpoint(self, endpoint):
        self.endpoint = endpoint

    def check_waypoint_treshold_reached(self):
        gps_data = self.gps.getValues()
        current_position = [gps_data[0], gps_data[1], gps_data[2]]
        desired_position = self.endpoint
        distance = sqrt((current_position[0] - desired_position[0])**2 + (current_position[1] - desired_position[1])**2 + (current_position[2] - desired_position[2])**2)
        if distance < self.waypoint_tolerance:
            return True
        else:
            return False
    
    def compute_desired_values(self):
        #desired_sideways = self.compute_desired_sideways()
        desired_yaw = self.compute_desired_yaw()
        if desired_yaw != 0:
            desired_forward = 0
        else:
            desired_forward = self.compute_desired_forward() 

        desired_altitude = self.compute_desired_altitude()
        #desired_forward = 0.3
        desired_sideways = 0
        return (desired_forward, desired_sideways, desired_yaw, desired_altitude)

    ## Compute the desired forward velocity
    def compute_desired_forward(self):
        gps_data = self.gps.getValues()
        current_position = [gps_data[0], gps_data[1], gps_data[2]]
        desired_position = self.endpoint

        # Compute the vector between the drone and the current waypoint
        desired_direction = np.array(desired_position) - np.array(current_position)
    
        # Calculate the length of the vector
        vector_length = np.linalg.norm(desired_direction)

        if vector_length < 0:
            desired_forward = -0.3
        elif vector_length > 0 and vector_length < 0.3:
            desired_forward = vector_length
        else:
            desired_forward = 0.3

        return desired_forward  # Multiply by SPEED_MULTIPLIER to slow down the drone

    ## Compute the desired sideways velocity
    def compute_desired_sideways(self):
        gps_data = self.gps.getValues()
        current_position = [gps_data[0], gps_data[1], gps_data[2]]
        desired_position = self.endpoint

        # Compute the vector between the drone and the current waypoint
        desired_direction = np.array(desired_position) - np.array(current_position)
    
        # Calculate the length of the vector
        vector_length = np.linalg.norm(desired_direction)

        # Normalize the vector without using explicit division
        if vector_length != 0:
            reciprocal_length = 1.0 / vector_length
            desired_direction *= reciprocal_length

        desired_sideways = desired_direction[1]
    
        return desired_sideways * SPEED_MULTIPLIER  # Multiply by SPEED_MULTIPLIER to slow down the drone

    ## Compute the desired yaw rate
    def compute_desired_yaw(self):
        gps_data = self.gps.getValues()
        current_orientation = self.imu.getRollPitchYaw()
        desired_position = self.endpoint

        desired_direction = np.array(desired_position) - np.array([gps_data[0], gps_data[1], gps_data[2]])
        current_direction = np.array([cos(current_orientation[2]), sin(current_orientation[2])])
        desired_yaw = atan2(desired_direction[1], desired_direction[0]) - atan2(current_direction[1], current_direction[0])

        if desired_yaw < -0.1:
            return -1
        elif desired_yaw > 0.1:
            return 1
        else:
            desired_yaw = 0
        return desired_yaw

    ## Compute the desired altitude
    def compute_desired_altitude(self):
        return self.endpoint[2]
    
    def generate_waypoints(self, start_position, waypoints, num_points=5):
        # Include start position as the first waypoint
        all_waypoints = [start_position]

        # Loop through each consecutive pair of waypoints
        for i in range(len(waypoints)):
            start = np.array(all_waypoints[-1])
            end = np.array(waypoints[i])
            # Generate num_points between start and end
            for j in range(1, num_points + 1):
                waypoint = start + (end - start) * (j / num_points)
                all_waypoints.append(waypoint.tolist())

        return all_waypoints