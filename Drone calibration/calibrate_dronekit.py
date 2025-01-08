# following code can be used for full calibration process
# Calibrations processed: ESC, 
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time


# setting up the connection:
drone_connect = '/dev/ttyACM0'
baud_rate = 115200 # default 57600

# connection establishment:
print(f"vehicle is connecting on: {drone_connect}")
V1drone = connect(drone_connect, baud=baud_rate, wait_ready=True)
#connected:
print(f"vehicle is connected through: {drone_connect}")

# first ensure that the vehicle is disarmed
# check if disarmed:
V1drone.armed = False

# function 1 - for ESC calibration:

# Wait for a few seconds to enter ESC calibration mode
print("Entering ESC calibration mode...")
V1drone.lush()
time.sleep(5)

# setting the throttle to minimum:
V1drone.channels.overrides = {'3', 1000}
print("Throttle set to minimum. Calibration complete.")
time.sleep(5)

V1drone.channels.overrides = {}

# function 2 - For GPS calibration:
print("Waiting for GPS lock...")
V1drone.parameters['GPS_TYPE'] = 1  # 1 for UBlox G

# Set the GPS baud rate if needed
V1drone.parameters['SERIAL3_BAUD'] = 115  # Match your GPS baud rate

print("GPS configured!")



# function 3 - For acceleration calibration:
# Send accelerometer calibration command
print("Calibrating accelerometer...")
V1drone.send_mavlink(V1drone.message_factory.command_long_encode(
    0, 0,    # target system, target component
    mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, # Command
    0,       # Confirmation
    1, 0, 0, 0, 0, 0, 0))  # Accelerometer calibration

V1drone.flush()
print("Accelerometer calibration initiated.")

# GPS testing:
print("GPS Fix:",V1drone.gps_0.fix_type)

print("Satellites Visible:", V1drone.gps_0.satellites_visible)

# verify the Calibration process:
print("Accelerometer Calibrated:", V1drone.parameters['CAL_ACC0_ID'])
print("GPS Status:", V1drone.gps_0.fix_type)



