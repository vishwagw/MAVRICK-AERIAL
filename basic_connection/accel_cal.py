from pymavlink import mavutil

# Create a MAVLink connection
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# Wait for a heartbeat from the flight controller
master.wait_heartbeat()

# Start accelerometer calibration
print("Starting accelerometer calibration...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
    0,
    1, 0, 0, 0, 0, 0, 0  # Accelerometer calibration
)

print("Follow the instructions to complete calibration.")
