import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import numpy as np

# Load the CSV file
file_path = '/home/kangle/Documents/FYP/gym-pybullet-drones/gym_pybullet_drones/examples/trajectory_1.csv'  # Update this path
trajectory_data = pd.read_csv(file_path)

# Extract x, y, z columns for processing
waypoints = trajectory_data[['x', 'y', 'z']].values

# Define the velocity and acceleration limits
vlim = np.array([1, 1, 1])  # velocity limits in each axis
alim = np.array([0.5, 0.5, 0.5])  # acceleration limits in each axis

# Create path from waypoints
path = ta.SplineInterpolator(np.linspace(0, 1, len(waypoints)), waypoints)

# Create velocity and acceleration constraints
pc_vel = constraint.JointVelocityConstraint(vlim)
pc_acc = constraint.JointAccelerationConstraint(alim)

# Setup the parameterization problem
instance = algo.TOPPRA([pc_vel, pc_acc], path, solver_wrapper='seidel')

# Compute the trajectory
jnt_traj = instance.compute_trajectory(0, 0)

# Sample the trajectory
N_samples = 1000
ss = np.linspace(0, jnt_traj.duration, N_samples)
qs = jnt_traj(ss)

# Extract the x, y, z components of the trajectory
x = qs[:, 0]
y = qs[:, 1]
z = qs[:, 2]

# Save the processed waypoints to a new CSV file
processed_waypoints = pd.DataFrame({
    'x': x,
    'y': y,
    'z': z
})
processed_file_path = '/home/kangle/Documents/FYP/gym-pybullet-drones/gym_pybullet_drones/examples/hello.csv'  # Update this path
processed_waypoints.to_csv(processed_file_path, index=False)

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the original waypoints
ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], color='red', label='Original Waypoints')

# Plot the processed trajectory
ax.plot(x, y, z, label='Processed Trajectory', color='blue')

# Labeling the axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Comparison of Original Waypoints and Processed Trajectory')

# Display the legend
ax.legend()

# Show the plot
plt.show()
