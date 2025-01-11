from dronekit import connect, VehicleMode

# setting up the connection:
drone_connect = '/dev/ttyACM0'
baud_rate = 115200 # default 57600

# connection establishment:
print(f"vehicle is connecting on: {drone_connect}")
V1drone = connect(drone_connect, baud=baud_rate, wait_ready=True)
#connected:
print(f"vehicle is connected through: {drone_connect}")

# Display vehicle state
print(f"Autopilot Firmware version: {V1drone.version}")
print(f"Global Location: {V1drone.location.global_frame}")
print(f"Battery: {V1drone.battery}")
print(f"Mode: {V1drone.mode.name}")

# Close connection
V1drone.close()
print("vehicle Connection is closed.")

