import serial
import codecs

serial_port = "COM5"  # Replace with the appropriate serial port
baud_rate = 115200  # Make sure it matches the baud rate set in the Arduino code
file_name = "C:/Users/ollem/OneDrive/Documents/pyFiles/VENTUS_data.csv"

# Open the serial connection
ser = serial.Serial(serial_port, baud_rate)

# Open the file in write mode to save the received data
with codecs.open(file_name, "w", encoding="utf-8", errors="ignore") as file:
    print("Receiving data...")
    while True:
        # Read a line of data from the serial port
        try:
            line = ser.readline().decode().rstrip('\r\n')
        except UnicodeDecodeError:
            continue
        if not line:
            break
        # Write the received data to the file
        file.write(line + '\n')
        # Print the received data to the console
        print(line)

print("Data received and saved to file")
ser.close()