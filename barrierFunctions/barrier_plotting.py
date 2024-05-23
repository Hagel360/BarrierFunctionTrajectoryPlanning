"""
Example plots for the barrier model.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from BarrierClass import Barrier as Bd

# create example trajectories
first_trajectory = np.array([[0.4,0.1, 0.0], [0.6,0.4, 0.3], [0.8,0.8, 0.8], [0.9, 1.2, 1.0], [0.8, 1.4, 1.5], [0.6, 1.7, 1.8]])
second_trajectory = np.array([[1.6,0.1, 0.0], [1.4,0.4, 0.5], [1.2,0.8, 0.7], [1.0,1.2, 0.8], [0.9, 1.4, 1.2], [0.7, 1.7, 0.9]])
third_trajectory = np.array([[0.5,0.1, 0.0], [0.7,0.3, 0.2], [0.8,0.6, 0.6], [1.0, 1.0, 1.1], [1.1, 0.6, 1.3], [1.4, 0.2, 0.7]])
#plot trajectories
fig = plt.figure()

ax1 = fig.add_subplot(1, 1, 1, projection='3d')
x = first_trajectory[:,0]
y = first_trajectory[:,1]
z = first_trajectory[:,2]
ax1.plot(x, y, z, 'bo-', label='Trajectory 1')
x = second_trajectory[:,0]
y = second_trajectory[:,1]
z = second_trajectory[:,2]
ax1.plot(x, y, z, 'ro-', label='Trajectory 2')
x = third_trajectory[:,0]
y = third_trajectory[:,1]
z = third_trajectory[:,2]
ax1.plot(x, y, z, 'go-', label='Trajectory 3')
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_zlabel('z')
ax1.legend()
plt.show()

new_first_trajectory = np.array([])
new_second_trajectory = np.array([])
new_third_trajectory = np.array([])
# add 50 points between each pair of points in the trajectory using linspace
for i in range(len(new_first_trajectory) - 1):
    new_left_trajectory = np.append(new_left_trajectory, np.linspace(first_trajectory[i], first_trajectory[i+1], num=5))
    new_right_trajectory = np.append(new_right_trajectory, np.linspace(third_trajectory[i], second_trajectory[i+1], num=5))
    new_third_trajectory = np.append(new_third_trajectory, np.linspace(third_trajectory[i], third_trajectory[i+1], num=5))
# initialize derivative barrier class
Bd = Bd()

first_trajectories = np.array([first_trajectory])
second_trajectories = np.array([second_trajectory])
third_trajectories = np.array([third_trajectory])

trajectories = np.array([first_trajectory, second_trajectory, third_trajectory])
# compute barrier potential for each trajectory

potentials = Bd.potential_value_sum([first_trajectory, second_trajectory, third_trajectory])

# create 3d plot with a heatmap for the potential values of the trajectories
# Plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot trajectories
for i, traj in enumerate(trajectories):
    ax.plot(traj[:,0], traj[:,1], traj[:,2], marker='o', label=f'Trajectory {i+1}')

# Plot potential values as heatmap
for i, traj in enumerate(trajectories):
    for j, point in enumerate(traj):
        ax.scatter(point[0], point[1], point[2], 
                   c=np.array([potentials[i, j]]), cmap='viridis',
                   s=100, edgecolors='k', linewidths=0.5, vmin=np.min(potentials), vmax=np.max(potentials))

# Color bar
cbar = plt.colorbar(ax.collections[0], ax=ax, orientation='vertical')
cbar.set_label('Potential Value')

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Trajectories with Potential Value Heatmap')
ax.legend()

plt.show()