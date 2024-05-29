import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from datetime import datetime

from helper_functions.barrier_class import Barrier

LINSPACEPOINTS = 5

class PeerToPeerClass:
    def __init__(self, drone_name):
        self.name = drone_name    
        self.drones = ["Crazyflie1", "Crazyflie2", "Crazyflie3"]
        # Drone trajectories
        self.drone1_trajectory = []
        self.drone2_trajectory = []
        self.drone3_trajectory = []

    def add_trajectory(self, name, trajectory):
        if name == "Crazyflie1":
            self.drone1_trajectory = trajectory
        if name == "Crazyflie2":
            self.drone2_trajectory = trajectory
        if name == "Crazyflie3":
            self.drone3_trajectory = trajectory

    def has_all_trajectories(self):
        return len(self.drone1_trajectory) > 0 and len(self.drone2_trajectory) > 0 and len(self.drone3_trajectory) > 0

    def get_trajectory(self):
        if self.name == "Crazyflie1":
            return self.drone1_trajectory
        if self.name == "Crazyflie2":
            return self.drone2_trajectory
        if self.name == "Crazyflie3":
            return self.drone3_trajectory

    # main loop
    # ensures that the drone has recieved all trajectories, then computes a new and updated trajectory
    def run(self):
        # hola HOOP RING FOR DRONES TO FLY THROUGH
        # Parameters
        radius = 3
        height = 1
        num_points = 10

        # Create the circular trajectory parallel to the x-axis
        theta = np.linspace(0, 2*np.pi, num_points)
        x = radius * np.cos(theta)  # Generating on the x-axis
        y = np.zeros_like(theta)
        z = radius * np.sin(theta)  # z-axis corresponds to height

        # Display the coordinates
        hula_hoop = np.column_stack([x, y, z])

        # Shift the hula hoop to the starting position
        hula_hoop[:, 0] += 0
        hula_hoop[:, 1] += -1.5
        hula_hoop[:, 2] += height

        barrier = Barrier(d_hat=0.3, eps=1.0, n=2)
        # use linspace to create a list of points between the drones point and trajectory
        # this is used to calculate the barrier function
        drone1_new_trajectory = np.array([])
        drone2_new_trajectory = np.array([])
        drone3_new_trajectory = np.array([])
        # add 5 points between each pair of points in the trajectory using linspace
        for i in range(len(self.drone1_trajectory) - 1):
            new_points = np.linspace(self.drone1_trajectory[i], self.drone1_trajectory[i+1], num=LINSPACEPOINTS, endpoint=False)
            if len(drone1_new_trajectory) == 0:
                drone1_new_trajectory = new_points
            else:
                drone1_new_trajectory = np.concatenate((drone1_new_trajectory, new_points))
        for i in range(len(self.drone2_trajectory) - 1):
            new_points = np.linspace(self.drone2_trajectory[i], self.drone2_trajectory[i+1], num=LINSPACEPOINTS, endpoint=False)
            if len(drone2_new_trajectory) == 0:
                drone2_new_trajectory = new_points
            else:
                drone2_new_trajectory = np.concatenate((drone2_new_trajectory, new_points))
        for i in range(len(self.drone3_trajectory) - 1):
            new_points = np.linspace(self.drone3_trajectory[i], self.drone3_trajectory[i+1], num=LINSPACEPOINTS, endpoint=False)
            if len(drone3_new_trajectory) == 0:
                drone3_new_trajectory = new_points
            else:
                drone3_new_trajectory = np.concatenate((drone3_new_trajectory, new_points))

        for i in range(200):
            # compute new trajectory using gradient descent
            self.drone1_trajectory = barrier.update_trajectory(drone1_new_trajectory, [drone2_new_trajectory, drone3_new_trajectory, hula_hoop], 0.1)
            self.drone2_trajectory = barrier.update_trajectory(drone2_new_trajectory, [drone1_new_trajectory, drone3_new_trajectory, hula_hoop], 0.1)
            self.drone3_trajectory = barrier.update_trajectory(drone3_new_trajectory, [drone1_new_trajectory, drone2_new_trajectory, hula_hoop], 0.1)

            drone1_new_trajectory = self.drone1_trajectory
            drone2_new_trajectory = self.drone2_trajectory
            drone3_new_trajectory = self.drone3_trajectory
