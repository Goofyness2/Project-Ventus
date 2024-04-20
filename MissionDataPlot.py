import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# File path
file_name = r"C:\Users\ollem\OneDrive\Documents\Project Ventus\VENTUS_data\GA-material\First_Test_of_Mission_Plots\Raw\Py_logger_10_Hz.csv"

# Load data from the file
data = np.genfromtxt(file_name, delimiter=',', skip_header=1)

def calculate_wind_indices(csv_file, delta_time):
    # Read CSV file into a DataFrame
    df = pd.read_csv(csv_file, encoding='latin-1')
    
    # Calculate mean wind speed (μ)
    mean_wind_speed = np.mean(np.sqrt(df[' Wind Velocity x [m/s]']**2 + df[' Wind Velocity y [m/s]']**2))
    
    # Calculate discrete derivative (ψ)
    dx = np.diff(df[' Wind Velocity x [m/s]'])
    dy = np.diff(df[' Wind Velocity y [m/s]'])
    psi = np.sqrt(np.sum(dx**2 + dy**2)) / (len(df) * delta_time)
    
    return mean_wind_speed, psi

mu, psi = calculate_wind_indices(file_name, 0.1)
print("Mean Wind Speed (μ):", mu, " [m/s]")
print("Discrete Derivative (ψ):", psi, "[m/s2]")

# Extracting data columns
altitude = data[:, 1]
temperature = data[:, 2]
pressure = data[:, 3]
wind_x = data[:, 4]
wind_y = data[:, 5]

# Create figure and axis
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

# Plotting pressure over altitude with temperature as color
ax1.scatter(pressure, altitude, c=temperature, cmap='RdYlBu_r', alpha=0.7)
cbar = plt.colorbar(ax1.collections[0], ax=ax1)
cbar.set_label('Temperature [°C]')
ax1.set_xlabel('Pressure [Pa]')
ax1.set_ylabel('Altitude [m.a.s.l]')
ax1.set_title('Pressure and Temperature over Altitudes')

# Plotting wind speed in the x-direction (red)
ax2.plot(wind_x, altitude, color='red', linewidth=2, label='x')

# Plotting wind speed in the y-direction (green)
ax2.plot(wind_y, altitude, color='green', linewidth=2, label='y')

mu_label = 'Wind Speed [m/s] \n\n Mean Wind Speed (μ): '
psi_label = ' [m/s] \n Discrete Derivative (ψ): '
last_label = ' [m/s2]'
label_x = mu_label + str(round(mu, 2)) + psi_label + str(round(psi, 2)) + last_label

# Set labels and title for the second subplot
ax2.set_xlabel(label_x)
ax2.set_ylabel('Altitude [m.a.s.l]')
ax2.set_title('Wind Speed over Altitude')

# Add legend to the second subplot
ax2.legend()

plt.tight_layout()

plt.show()
