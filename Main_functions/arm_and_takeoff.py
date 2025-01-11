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


# arm and take off function:
# arm: this is for starting the drone for fully powered
# take off: taking off from gorund level
def arm_and_takeoff(aTargetAltitude):
    print("V1 drone is starting to arm...")
    while not V1D.is_armable:
        # sleep for one sec:
        time.sleep(1)
    
    print('Motors are starting to rotate..')
    print("Motors have started..")
    V1D.mode = VehicleMode("GUIDED") # This is th guided mode commoand from ardupilot mission planner
    # arming:
    V1D.armed = True
    # confirming armed:
    while not V1D.armed:
        time.sleep(1)
    
    # start to taking off:
    V1D.simple_takeoff(aTargetAltitude)
    # this will take the drone for a specific given altitde level.

    # first wait until the drone reach target altitude:
    # then any other function can be executed:
    # accuracy for target altitude : 0.95
    # just before reacing the target altitude function should be breaked
    while True:
        if V1D.location.global_relative_frame.alt  >= aTargetAltitude*0.95:
            break
        time.sleep(1)

# initializing the function:
# 10 is the target altitude in ft
arm_and_takeoff(6)








    

