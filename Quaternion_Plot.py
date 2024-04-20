import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import atan2, asin, degrees

# Initialize serial communication with Arduino
ser = serial.Serial('COM11', 2000000)  # Adjust COM port accordingly
ser.flush()

# Initialize figure and axis for 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

def quaternion_to_euler(w, x, y, z):
    pitch = degrees(asin(2.0 * (w * y - z * x)))
    yaw = degrees(atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    roll = degrees(atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
    return pitch, yaw, roll

# Function to update plot
def update_plot():
    # Read quaternion from serial
    if ser.in_waiting > 0:
        quaternion_str = ser.readline().decode('utf-8').strip()
        quaternion = [float(val) for val in quaternion_str.split(',')]
        w, x, y, z = quaternion
        
        # Convert quaternion to rotation matrix
        R = np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                      [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                      [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]])
        
        # Define x, y, z vectors
        x_vec = R.dot(np.array([1, 0, 0]))
        y_vec = R.dot(np.array([0, 1, 0]))
        z_vec = R.dot(np.array([0, 0, 1]))
        
        # Plot x, y, z vectors
        ax.clear()
        ax.quiver(0, 0, 0, x_vec[0], x_vec[1], -x_vec[2], color='r', label='X-axis')
        ax.quiver(0, 0, 0, y_vec[0], y_vec[1], -y_vec[2], color='g', label='Y-axis')
        ax.quiver(0, 0, 0, z_vec[0], z_vec[1], -z_vec[2], color='b', label='Z-axis')


        # Convert quaternion to Euler angles
        pitch, yaw, roll = quaternion_to_euler(w, x, y, z)

        # Add text annotation for pitch, yaw, roll
        text = f'Pitch: {pitch:.2f} deg\nYaw: {yaw:.2f} deg\nRoll: {roll:.2f} deg'
        ax.text2D(-0.2, 0.0, text, transform=ax.transAxes, ha='center', va='center')
        
        # Set axis limits
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        
        # Set labels and title
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Quaternion Orientation')
        
        # Show legend
        ax.legend()
        
        # Show plot
        plt.pause(0.01)  # Pause to allow plot to update

# Continuous update of the plot
try:
    while True:
        update_plot()
except KeyboardInterrupt:
    # Close serial connection
    ser.close()
