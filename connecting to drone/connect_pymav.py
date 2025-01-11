from pymavlink import mavutil

# first we are establishing the connection:
drone_connect = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# now we havr to wait for the heartbeat to confiem the connection:
# heartbeat in drone is a protocol that informs other compnents on the mavlink network of a drone existence, as well as its system and components, ID, flightmode and more.
print("Waitin for heartbeat...")
drone_connect.wait_heartbeat()
print("Heartbeat received from system (system %u component %u)" % (drone_connect.target_system, drone_connect.target_component))

# Example: Requesting vehicle's attitude
drone_connect.mav.request_data_stream_send(
    drone_connect.target_system,
    drone_connect.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    1,  # Request rate (Hz)
    1   # Start streaming
)

# Read and print messages
while True:
    msg = drone_connect.recv_match(blocking=True)
    if not msg:
        continue
    print(msg)

S