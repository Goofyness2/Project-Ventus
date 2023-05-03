import matplotlib.pyplot as plt
import numpy as np
import re
import serial
from collections import deque
import threading
import sys
from matplotlib.animation import FuncAnimation as funcAnimation

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

    # Plot the data
    xs = [d[0] for d in data]
    ys = [d[1] for d in data]
    zs = [d[2] for d in data]
    ax.plot(xs, ys, zs)

    plt.draw()

# Define a custom event handler function for the "close event" of the plot window
def on_close(event):
    global plot_open
    plot_open = False
    print("Plot window closed.")
    ser.close()
    sys.exit()

# Register the custom event handler function using fig.canvas.mpl_connect()
fig.canvas.mpl_connect('close_event', on_close)

# Start a separate thread to read data from the serial port
thread = threading.Thread(target=read_data)
thread.start()

# Update the plot using FuncAnimation
ani = funcAnimation(fig, update_plot, interval=100)

# Set the global flag to indicate that the plot window is still open
plot_open = True

# Wait until the plot window is closed
while plot_open:
    plt.pause(1)
    print(arr[-1])
