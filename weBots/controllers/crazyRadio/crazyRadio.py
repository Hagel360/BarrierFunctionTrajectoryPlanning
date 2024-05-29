"""
Main program for the CrazyRadio
"""
from controller import Robot
from radio_controller import radio_controller
from barrier_class import Barrier
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


from datetime import datetime

COMMUNICATION_MODE = "crazyradio"
##COMMUNICATION_MODE = "peer_to_peer"
LINSPACEPOINTS = 5

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


# #plot the hula_hoop to verify its shape
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.plot(x, y, z, c='r', marker='o', label='hula_hoop')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# plt.show()

if __name__ == "__main__":
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    dongle = radio_controller(robot)
    sensor_read_last_time = robot.getTime()
    all_trajectory_received = False
    # list of active drones
    drones = ["Crazyflie1", "Crazyflie2", "Crazyflie3"]
    # arrays for drone coordinats, to be plotted
    drone1_coords = []
    drone2_coords = []
    drone3_coords = []
    # Drone trajectories
    drone1_trajectory = np.array([])
    drone2_trajectory = np.array([])
    drone3_trajectory = np.array([])

    print("Starting simulation")

    # pack testname and testpacket into a struct and send it
    # every time step, check if there are packets in the queue
    while robot.step(timestep) != -1:
        # check if there are packets in the queue every 2 second after first package is received
        if len(drones) > 0:
            if dongle.receiver.getQueueLength() > 0:
                receivedPacket = dongle.receive_packet()
                if receivedPacket[1] == "POS":
                    if receivedPacket[0] == "Crazyflie1":
                        drone1_coords.append(receivedPacket[2])
                    elif receivedPacket[0] == "Crazyflie2":
                        drone2_coords.append(receivedPacket[2])
                    elif receivedPacket[0] == "Crazyflie3":
                        drone3_coords.append(receivedPacket[2])
                if receivedPacket[1] == "NACK":
                    drones.remove(receivedPacket[0])
                if receivedPacket[1] == "ACK":
                    if receivedPacket[0] not in drones:
                        drones.append(receivedPacket[0])
                # if received a trajectory, save it in respective array
                # if all three trajectories are received, compute new trajectories
                # return them to the drones
                if receivedPacket[1] == "TRAJ" and all_trajectory_received == False:
                    if receivedPacket[0] == "Crazyflie1" and len(drone1_trajectory) == 0:
                        drone1_trajectory = np.array(receivedPacket[2])
                    if receivedPacket[0] == "Crazyflie2" and len(drone2_trajectory) == 0:
                        drone2_trajectory = np.array(receivedPacket[2])
                    if receivedPacket[0] == "Crazyflie3" and len(drone3_trajectory) == 0:
                        drone3_trajectory = np.array(receivedPacket[2])

                    if len(drone1_trajectory) > 0 and len(drone2_trajectory) > 0 and len(drone3_trajectory) > 0:
                        barrier = Barrier(d_hat=0.3, eps=1.0, n=2)
                        # use linspace to create a list of points between the drones point and trajectory
                        # this is used to calculate the barrier function
                        drone1_new_trajectory = np.array([])
                        drone2_new_trajectory = np.array([])
                        drone3_new_trajectory = np.array([])
                        # add 5 points between each pair of points in the trajectory using linspace
                        for i in range(len(drone1_trajectory) - 1):
                            new_points = np.linspace(drone1_trajectory[i], drone1_trajectory[i+1], num=LINSPACEPOINTS, endpoint=False)
                            if len(drone1_new_trajectory) == 0:
                                drone1_new_trajectory = new_points
                            else:
                                drone1_new_trajectory = np.concatenate((drone1_new_trajectory, new_points))
                        for i in range(len(drone2_trajectory) - 1):
                            new_points = np.linspace(drone2_trajectory[i], drone2_trajectory[i+1], num=LINSPACEPOINTS, endpoint=False)
                            if len(drone2_new_trajectory) == 0:
                                drone2_new_trajectory = new_points
                            else:
                                drone2_new_trajectory = np.concatenate((drone2_new_trajectory, new_points))
                        for i in range(len(drone3_trajectory) - 1):
                            new_points = np.linspace(drone3_trajectory[i], drone3_trajectory[i+1], num=LINSPACEPOINTS, endpoint=False)
                            if len(drone3_new_trajectory) == 0:
                                drone3_new_trajectory = new_points
                            else:
                                drone3_new_trajectory = np.concatenate((drone3_new_trajectory, new_points))
                        # save the new trajectories in the respective arrays, so we can create a gif of the simulation later
                        drone1_gif_trajectory = np.array([drone1_new_trajectory])
                        drone2_gif_trajectory = np.array([drone2_new_trajectory])
                        drone3_gif_trajectory = np.array([drone3_new_trajectory])

                        drone1_old_trajectory = drone1_new_trajectory
                        drone2_old_trajectory = drone2_new_trajectory
                        drone3_old_trajectory = drone3_new_trajectory

                        for i in range(200):
                            # compute new trajectory using gradient descent
                            drone1_trajectory = barrier.update_trajectory(drone1_new_trajectory, [drone2_new_trajectory, drone3_new_trajectory, hula_hoop], 0.1)
                            drone2_trajectory = barrier.update_trajectory(drone2_new_trajectory, [drone1_new_trajectory, drone3_new_trajectory, hula_hoop], 0.1)
                            drone3_trajectory = barrier.update_trajectory(drone3_new_trajectory, [drone1_new_trajectory, drone2_new_trajectory, hula_hoop], 0.1)

                            drone1_new_trajectory = drone1_trajectory
                            drone2_new_trajectory = drone2_trajectory
                            drone3_new_trajectory = drone3_trajectory

                            # append to gif trajectory
                            drone1_gif_trajectory = np.append(drone1_gif_trajectory, [drone1_new_trajectory], axis=0)
                            drone2_gif_trajectory = np.append(drone2_gif_trajectory, [drone2_new_trajectory], axis=0)
                            drone3_gif_trajectory = np.append(drone3_gif_trajectory, [drone3_new_trajectory], axis=0)
                            
                        # send the new trajectories to the drones
                        if COMMUNICATION_MODE == "crazyradio":
                            print("Sending new trajectories to drones")
                            dongle.send_packet("Crazyflie1", "TRAJ", drone1_trajectory.tolist())
                            dongle.send_packet("Crazyflie2", "TRAJ", drone2_trajectory.tolist())
                            dongle.send_packet("Crazyflie3", "TRAJ", drone3_trajectory.tolist())
                            all_trajectory_received = True

        else:
            break
    
    # plot the drone trajectories in 3d with line between them the points
    # each drone has different color
    if COMMUNICATION_MODE == "crazyradio":
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot([hula_hoop[i][0] for i in range(len(hula_hoop))], [hula_hoop[i][1] for i in range(len(hula_hoop))], [hula_hoop[i][2] for i in range(len(hula_hoop))], c='y', marker='o', label='hula_hoop')
        ax.plot([drone1_old_trajectory[i][0] for i in range(len(drone1_old_trajectory))], [drone1_old_trajectory[i][1] for i in range(len(drone1_old_trajectory))], [drone1_old_trajectory[i][2] for i in range(len(drone1_old_trajectory))], c='c', marker='o', label='Crazyflie1 old trajectory')
        ax.plot([drone2_old_trajectory[i][0] for i in range(len(drone2_old_trajectory))], [drone2_old_trajectory[i][1] for i in range(len(drone2_old_trajectory))], [drone2_old_trajectory[i][2] for i in range(len(drone2_old_trajectory))], c='m', marker='o', label='Crazyflie2 old trajectory')
        ax.plot([drone3_old_trajectory[i][0] for i in range(len(drone3_old_trajectory))], [drone3_old_trajectory[i][1] for i in range(len(drone3_old_trajectory))], [drone3_old_trajectory[i][2] for i in range(len(drone3_old_trajectory))], c='y', marker='o', label='Crazyflie3 old trajectory')
        ax.plot([drone1_trajectory[i][0] for i in range(len(drone1_trajectory))], [drone1_trajectory[i][1] for i in range(len(drone1_trajectory))], [drone1_trajectory[i][2] for i in range(len(drone1_trajectory))], c='r', marker='o', label='Crazyflie1 new trajectory')
        ax.plot([drone2_trajectory[i][0] for i in range(len(drone2_trajectory))], [drone2_trajectory[i][1] for i in range(len(drone2_trajectory))], [drone2_trajectory[i][2] for i in range(len(drone2_trajectory))], c='g', marker='o', label='Crazyflie2 new trajectory')
        ax.plot([drone3_trajectory[i][0] for i in range(len(drone3_trajectory))], [drone3_trajectory[i][1] for i in range(len(drone3_trajectory))], [drone3_trajectory[i][2] for i in range(len(drone3_trajectory))], c='b', marker='o', label='Crazyflie3 new trajectory')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        # add timestamp to string, to save all simulations
        savestring = "img/drone_trajectories3d_simulated" + datetime.now().strftime("%H%M%S") + ".png"
        plt.savefig(savestring)
        plt.show()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot([hula_hoop[i][0] for i in range(len(hula_hoop))], [hula_hoop[i][1] for i in range(len(hula_hoop))], [hula_hoop[i][2] for i in range(len(hula_hoop))], c='y', marker='o', label='hula_hoop')
    ax.plot([drone1_coords[i][0] for i in range(len(drone1_coords))], [drone1_coords[i][1] for i in range(len(drone1_coords))], [drone1_coords[i][2] for i in range(len(drone1_coords))], c='r')    
    ax.plot([drone1_coords[i][0] for i in range(len(drone1_coords))], [drone1_coords[i][1] for i in range(len(drone1_coords))], [drone1_coords[i][2] for i in range(len(drone1_coords))], c='r')
    ax.plot([drone2_coords[i][0] for i in range(len(drone2_coords))], [drone2_coords[i][1] for i in range(len(drone2_coords))], [drone2_coords[i][2] for i in range(len(drone2_coords))], c='g')
    ax.plot([drone3_coords[i][0] for i in range(len(drone3_coords))], [drone3_coords[i][1] for i in range(len(drone3_coords))], [drone3_coords[i][2] for i in range(len(drone3_coords))], c='b')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # add barrier values
    ax.text2D(0.05, 0.95, "Barrier values: d_hat=0.3, eps=1.0, n=2", transform=ax.transAxes)
    # add timestamp to string, to save all simulations
    savestring = "img/drone_positions3d_simulated" + datetime.now().strftime("%H%M%S") + ".png"
    plt.savefig(savestring)
    plt.show()

    if COMMUNICATION_MODE == "crazyradio":
        # plot the gif of the drone trajectories in 3d with line between them the points
        # each drone has different color
        fig = plt.figure()
        ims = []
        for i in range(len(drone1_gif_trajectory)):
            im0 = plt.plot(drone1_gif_trajectory[i][:,0], drone1_gif_trajectory[i][:,1], c='r', marker='o', label='Crazyflie1 new trajectory')
            im1 = plt.plot(drone2_gif_trajectory[i][:,0], drone2_gif_trajectory[i][:,1], c='g', marker='o', label='Crazyflie2 new trajectory')
            im2 = plt.plot(drone3_gif_trajectory[i][:,0], drone3_gif_trajectory[i][:,1], c='b', marker='o', label='Crazyflie3 new trajectory')
            ims.append(im0 + im1 + im2)
        savestring = "img/drone_trajectories3d_simulated" + datetime.now().strftime("%H%M%S") + ".gif"
        ani = animation.ArtistAnimation(fig, ims, interval=50, blit=True,
                                    repeat_delay=1000)
        ani.save(savestring, writer='ffmpeg', fps=30)
        plt.show()