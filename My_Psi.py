import pandas as pd
import numpy as np

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

# Example usage
csv_file = r"C:\Users\ollem\OneDrive\Documents\Project Ventus\VENTUS_data\Py_logger_test267.csv"
delta_time = 0.01  # Adjust this according to your time step in seconds

mu, psi = calculate_wind_indices(csv_file, delta_time)
print("Mean Wind Speed (μ):", mu, " [m/s]")
print("Discrete Derivative (ψ):", psi, "[m/s2]")
