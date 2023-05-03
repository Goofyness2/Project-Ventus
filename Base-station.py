import matplotlib.pyplot as plt
import numpy as np
import re
import serial
from collections import deque
import threading
import time
from matplotlib.animation import FuncAnimation

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

# Set the axis limits
# ax.set_xlim(-30, 30)
# ax.set_ylim(-30, 30)
# ax.set_zlim(-30, 30)

# Use a deque to store the data
arr = deque(maxlen=1000)

def read_data():
    while True:
        line_data = ser.readline().decode().strip()

        if re.match(pattern, line_data):
            row = np.array(line_data.split(","), dtype=float)
            arr.append(row)

def update_plot(frame):
    # Make a copy of the deque to avoid "deque mutated during iteration" error
    data = deque(arr)

    # Clear the plot
    ax.clear()

    # Set the axis labels
    ax.set_xlabel('Long (m)')
    ax.set_ylabel('Lat (m)')
    ax.set_zlabel('Alt (m)')

    # Set the axis limits
    # ax.set_xlim(-30, 30)
    # ax.set_ylim(-30, 30)
    # ax.set_zlim(-30, 30)

    # Plot the data
    xs = [d[0] for d in data]
    ys = [d[1] for d in data]
    zs = [d[2] for d in data]
    ax.scatter(xs, ys, zs)

    # Check if the plot window is still open
    if not plt.fignum_exists(fig.number):
        print("Plot window closed.")
        ser.close()
        sys.exit()

    plt.draw()

# Start a separate thread to read data from the serial port
thread = threading.Thread(target=read_data)
thread.start()

# Update the plot using FuncAnimation
ani = matplotlib.animation.FuncAnimation(fig, update_plot, interval=100)
plt.show()
