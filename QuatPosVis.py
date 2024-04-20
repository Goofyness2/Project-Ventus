import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion
import math

data = pd.read_csv(r'C:\Users\ollem\OneDrive\Documents\Project Ventus\VENTUS_data\Py_logger_test267.csv', encoding='latin1')

quaternions = data[[' q0', ' q1', ' q2', ' q3']].to_numpy()
positions = data[[' Kalman x [m]', ' Kalman y [m]', ' Kalman z [m]']].to_numpy()

# Step 4: Visualize Data
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot position as a black line
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], c='k', linestyle='-', linewidth=1, label='Position')

# Plot position points
ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], c='k', marker='o', label='Position')

# Specify interval for displaying vectors
display_interval = 10

# Plot orientation axes
for i, pos in enumerate(positions):
    if i % display_interval == 0:

        w = quaternions[i, 0]
        x = quaternions[i, 1]
        y = quaternions[i, 2]
        z = quaternions[i, 3]
        
        # Convert quaternion to rotation matrix
        R = np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                      [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                      [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]])
        
        # Define x, y, z vectors
        x_vec = R.dot(np.array([1, 0, 0]))
        y_vec = R.dot(np.array([0, 1, 0]))
        z_vec = R.dot(np.array([0, 0, 1]))
        
        
        # Plot x, y, z vectors
        ax.quiver(positions[i, 0], positions[i, 1], positions[i, 2], positions[i, 0], positions[i, 1], positions[i, 2] + 1, color='r')
        #ax.quiver(positions[i, 0], positions[i, 1], positions[i, 2], positions[i, 0] + y_vec[0], positions[i, 1] + y_vec[1], positions[i, 2] - y_vec[2], color='g')
        #ax.quiver(positions[i, 0], positions[i, 1], positions[i, 2], positions[i, 0] + z_vec[0], positions[i, 1] + z_vec[1], positions[i, 2] - z_vec[2], color='b')

# Set labels and legend
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

plt.show()
