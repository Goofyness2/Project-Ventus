import numpy as np
import matplotlib.pyplot as plt

# File path
file_name = r"C:\Users\ollem\OneDrive\Documents\Project Ventus\VENTUS_data\Py_logger_test267.csv"

# Load data from the file
data = np.genfromtxt(file_name, delimiter=',', skip_header=1)

# Extracting data columns
altitude = data[:, 7]
temperature = data[:, 1]
pressure = data[:, 2]
wind_x = data[:, 3]
wind_y = data[:, 4]

# Create figure and axis
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

# Plotting pressure over altitude with temperature as color
ax1.scatter(pressure, altitude, c=temperature, cmap='RdYlBu_r', alpha=0.7)
cbar = plt.colorbar(ax1.collections[0], ax=ax1)
cbar.set_label('Temperature [°C]')
ax1.set_xlabel('Pressure [Pa]')
ax1.set_ylabel('Altitude [m.a.s.l]')
ax1.set_title('Pressure and Temperature over Altitude')

# Plotting wind speed in the x-direction (red)
ax2.plot(wind_x, altitude, color='red', linewidth=2, label='x')

# Plotting wind speed in the y-direction (green)
ax2.plot(wind_y, altitude, color='green', linewidth=2, label='y')

# Set labels and title for the second subplot
ax2.set_xlabel('Wind Speed [m/s]')
ax2.set_ylabel('Altitude [m.a.s.l]')
ax2.set_title('Wind Speed over Altitude')

# Add legend to the second subplot
ax2.legend()

plt.show()
