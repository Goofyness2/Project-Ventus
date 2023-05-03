import matplotlib.pyplot as plt
import numpy as np
import re
import serial
import sys

pattern = r'^-?\d+\.\d+,-?\d+\.\d+,-?\d+\.\d+$'

import matplotlib
matplotlib.use('Qt5Agg')

# Set up the serial connection
ser = serial.Serial('/dev/ttyUSB0', 115200)

# Set up the plot
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Set the axis labels
ax.set_xlabel('Long (m)')
ax.set_ylabel('Lat (m)')
ax.set_zlabel('Alt (m)')

arr = np.empty((0, 3), float)

while True:
    # Read a line of data from the serial port
    line_data = ser.readline().decode().strip()

    if re.match(pattern, line_data):
        row = np.array(line_data.split(","), dtype=float)
        arr = np.vstack([arr, row])

    ax.scatter(arr[:, 0], arr[:, 1], arr[:, 2])

    plt.pause(0.01)

    # Check if the plot window is still open
    if not plt.fignum_exists(fig.number):
        print("Plot window closed.")
        ser.close()
        sys.exit()

    plt.show(block=False)  # Set block to False to avoid blocking the program
