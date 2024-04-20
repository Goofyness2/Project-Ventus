import matplotlib.pyplot as plt
import csv
import numpy as np

MARGINAL = 0.25

# Load data from CSV file
filename = r'C:\Users\ollem\OneDrive\Documents\Project Ventus\VENTUS_data\Py_logger_test313.csv'

# Define lists to store data
time = []
acceleration_x = []
acceleration_y = []
acceleration_z = []
kalman_x = []
kalman_y = []
kalman_z = []
gps_delta_x = []
gps_delta_y = []
bmp_altitude = []
velocity_x = []
velocity_y = []
velocity_z = []
quaternion_r = []
quaternion_i = []
quaternion_j = []
quaternion_k = []

# Read data from CSV
with open(filename, 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)  # skip header
    for row in csvreader:
        if len(row) == 23:
            time.append(float(row[0]))
            kalman_x.append(float(row[5]))
            kalman_y.append(float(row[6]))
            kalman_z.append(float(row[7]))
            acceleration_x.append(float(row[11]))
            acceleration_y.append(float(row[12]))
            acceleration_z.append(float(row[13]))
            velocity_x.append(float(row[8]))
            velocity_y.append(float(row[9]))
            velocity_z.append(float(row[10]))
            quaternion_r.append(float(row[14]))
            quaternion_i.append(float(row[15]))
            quaternion_j.append(float(row[16]))
            quaternion_k.append(float(row[17]))
            gps_delta_x.append(float(row[20]))
            gps_delta_y.append(float(row[21]))
            bmp_altitude.append(float(row[22]))

plt.figure(figsize=(12, 8))
plt.subplot(4, 1, 1)
plt.plot(time, acceleration_x, color='red', label='Acceleration x')
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [m/s^2]')
plt.title('Acceleration x over Time')
plt.legend()

plt.subplot(4, 1, 2)
plt.plot(time, acceleration_y, color='green', label='Acceleration y')
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [m/s^2]')
plt.title('Acceleration y over Time')
plt.legend()

plt.subplot(4, 1, 3)
plt.plot(time, acceleration_z, color='blue', label='Acceleration z')
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [m/s^2]')
plt.title('Acceleration z over Time')
plt.legend()

acceleration_mag = np.sqrt(np.array(acceleration_x)**2 + np.array(acceleration_y)**2 + np.array(acceleration_z)**2) / 9.82

plt.subplot(4, 1, 4)
plt.plot(time, acceleration_mag, color='black', label='Acceleration Magnitude')
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [g]')
plt.title('Acceleration Magnitude over Time')
plt.legend()
plt.tight_layout()

# Plot 2: Kalman x and GPS Delta x, Kalman y and GPS Delta y, Kalman z and BMP Altitude
plt.figure(figsize=(12, 8))
plt.subplot(3, 1, 1)
plt.plot(time, kalman_x, color='red', label='Kalman x')
plt.plot(time, gps_delta_x, color='black', label='GPS Delta x')
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.ylim(min(gps_delta_x) - MARGINAL * (max(gps_delta_x) - min(gps_delta_x)), max(gps_delta_x)  + MARGINAL * (max(gps_delta_x) - min(gps_delta_x)))
plt.title('Kalman x vs GPS Delta x')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, kalman_y, color='green', label='Kalman y')
plt.plot(time, gps_delta_y, color='black', label='GPS Delta y')
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.ylim(min(gps_delta_y) - MARGINAL * (max(gps_delta_y) - min(gps_delta_y)), max(gps_delta_y)  + MARGINAL * (max(gps_delta_y) - min(gps_delta_y)))
plt.title('Kalman y vs GPS Delta y')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, kalman_z, color='blue', label='Kalman z')
plt.plot(time, bmp_altitude, color='black', label='BMP Altitude')
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.title('Kalman z vs BMP Altitude')
plt.ylim(min(bmp_altitude) - MARGINAL * (max(bmp_altitude) - min(bmp_altitude)), max(bmp_altitude)  + MARGINAL * (max(bmp_altitude) - min(bmp_altitude)))
plt.legend()
plt.tight_layout()

# Plot 3: Velocity in all three axes over time
plt.figure(figsize=(12, 8))
plt.subplot(3, 1, 1)
plt.plot(time, velocity_x, color='red', label='Velocity x')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.title('Velocity x over Time')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, velocity_y, color='green', label='Velocity y')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.title('Velocity y over Time')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, velocity_z, color='blue', label='Velocity z')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.title('Velocity z over Time')
plt.legend()
plt.tight_layout()

# Plot 4: Quaternion data
plt.figure(figsize=(12, 8))
plt.plot(time, quaternion_r, color='red', label='Quaternion r')
plt.plot(time, quaternion_i, color='green', label='Quaternion i')
plt.plot(time, quaternion_j, color='blue', label='Quaternion j')
plt.plot(time, quaternion_k, color='orange', label='Quaternion k')
plt.xlabel('Time [s]')
plt.ylabel('Quaternion')
plt.title('Quaternion over Time')
plt.legend()
plt.tight_layout()

plt.show()
