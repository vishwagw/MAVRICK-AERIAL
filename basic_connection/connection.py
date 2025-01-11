from dronekit import connect

# Replace '/dev/ttyUSB0' with your specific port
connection_string = '/dev/ttyUSB0'
baud_rate = 115200  # Match this to your flight controller's baud rate

print("Connecting to vehicle...")
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
print("Connected to vehicle!")
