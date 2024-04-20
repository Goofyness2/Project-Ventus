import matplotlib.pyplot as plt
import pandas as pd

# Read the wind data from the CSV file
data = pd.read_csv(r'C:\Users\ollem\OneDrive\Documents\Project Ventus\VENTUS_data\Py_logger_test217.csv')

# Extract wind speed in x and y directions and altitude
wind_speed_x = data['windSpeedX']
wind_speed_y = data['windSpeedY']
altitude = data['altitude']

# Create two subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

# Plot wind speed in x-direction
ax1.plot(wind_speed_x, altitude, color='red')
ax1.set_title('Wind Speed X vs Altitude')
ax1.set_xlabel('Wind Speed X')
ax1.set_ylabel('Altitude')

# Plot wind speed in y-direction
ax2.plot(wind_speed_y, altitude, color='green')
ax2.set_title('Wind Speed Y vs Altitude')
ax2.set_xlabel('Wind Speed Y')
ax2.set_ylabel('Altitude')

# Display the plots
plt.tight_layout()
plt.show()
