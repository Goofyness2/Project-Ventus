import matplotlib.pyplot as plt
import numpy as np
import re
import serial

pattern = r'^-?\d+\.\d+,-?\d+\.\d+,-?\d+\.\d+$'

import matplotlib
matplotlib.use('Qt5Agg')


# Set up the serial connection
ser = serial.Serial('/dev/ttyUSB0', 115200)

# Set up the plot
plt.ion()
ax = plt.axes(projection = "3d")

# Set the axis labels
ax.set_xlabel('Long (m)')
ax.set_ylabel('Lat (m)')
ax.set_zlabel('Alt (m)')

# Set the axis limits
ax.set_xlim(-30, 30)
ax.set_ylim(-30, 30)
ax.set_zlim(-30, 30)

# # Initialize the data
# xdata = []
# ydata = []

arr = np.empty((0, 3), float)

# def on_close(event):
#     print('Plot window closed.')
#     plt.close()  # Close the window programatically
#     exit()  # Terminate the program

# fig.canvas.mpl_connect('close_event', on_close)

# Loop indefinitely
while True:
    # Read a line of data from the serial port
    line_data = ser.readline().decode().strip()

    if re.match(pattern, line_data):
        row = np.array(line_data.split(","), dtype=float)
        arr = np.vstack([arr, row])
    
    ax.scatter(arr[:, 0], arr[:, 1], arr[:, 2])

    # # Parse the data as an integer
    # value = float(line_data)

    # # Add the data to the plot
    # xdata.append(len(xdata))
    # ydata.append(value)
    # line.set_data(np.array(xdata), np.array(ydata))

    # # Update the plot
    # ax.set_xlim(min(xdata), max(xdata) + 1)
    # plt.draw()
    # plt.show()
    plt.pause(0.01)
