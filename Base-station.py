import serial
import matplotlib.pyplot as plt
import numpy as np

import matplotlib
matplotlib.use('Qt5Agg')  # Replace 'Qt5Agg' with the backend of your choice


# Set up the serial connection
ser = serial.Serial('/dev/ttyUSB0', 115200)

# Set up the plot
plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [])

# Set the axis labels
ax.set_xlabel('Time (s)')
ax.set_ylabel('Value')

# Set the axis limits
ax.set_xlim(0, 10)
ax.set_ylim(0, 1023)

# Initialize the data
xdata = []
ydata = []

# Loop indefinitely
while True:
    # Read a line of data from the serial port
    line_data = ser.readline().decode().strip()

    # Parse the data as an integer
    value = int(line_data)

    # Add the data to the plot
    xdata.append(len(xdata))
    ydata.append(value)
    line.set_data(np.array(xdata), np.array(ydata))

    # Update the plot
    plt.draw()
    plt.pause(0.01)
