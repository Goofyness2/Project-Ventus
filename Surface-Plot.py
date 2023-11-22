import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pandas as pd
from scipy.interpolate import griddata

# Load the data from the CSV file
data = pd.read_csv(r'C:\Users\ollem\OneDrive\Documents\Project Ventus\VENTUS_data\KF_Tune_log_8.csv')

# Extract X, Y, and Z values from the CSV data
X = data['Q'].values
Y = data['R'].values
Z = data['e'].values

# Create a grid of desired dimensions (e.g., 40x40)s
grid_x, grid_y = np.meshgrid(np.linspace(X.min(), X.max(), 40), np.linspace(Y.min(), Y.max(), 40))

# Interpolate the Z values onto the grid
grid_z = griddata((X, Y), Z, (grid_x, grid_y), method='linear')

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Set limits for the X, Y, and Z axes
ax.set_xlim(X.min(), X.max())
ax.set_ylim(Y.min(), Y.max())
ax.set_zlim(Z.min(), Z.max())

# Create a surface plot
surf = ax.plot_surface(grid_x, grid_y, grid_z, cmap='viridis')

# Add labels to the axes
ax.set_xlabel('Q')
ax.set_ylabel('R')
ax.set_zlabel('e (m)')

# Add a color bar
fig.colorbar(surf)

# Show the plot
plt.tight_layout()
plt.show()
