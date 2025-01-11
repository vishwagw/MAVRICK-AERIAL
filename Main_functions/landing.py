# libs:
import time
import gpio as io
from __future__ import print_function
# connect pymavlink
from pymavlink import mavutil
# connect dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

# connecting the program into the drone via connect port:
V1D = connect('./', wait_ready=True, baud=921000)

# this is the function for landing back:
def land():
    print("V1Drone is ready to land back...")
    V1D.mode = VehicleMode("LAND")
    print("Drone is landing")
    time.sleep(1)

    print("Drone has landed safely. ending mission")
    time.sleep(1)

    # finishing the mission:
    print("mission succesfully completed...")
    time.sleep(1)

# initialize the function:
land()
