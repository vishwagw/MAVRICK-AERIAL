# libs:
import time
import gpio as io
from __future__ import print_function
# connect pymavlink
from pymavlink import mavutil
# connect dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from drone_velocity import velocity
# connecting the program into the drone via connect port:
V1D = connect('./', wait_ready=True, baud=921000)

# function for basic ascend (go up) to a target location:
def ascend(aTargetAltitude):
    while V1D.location.global_relative_frame.alt <= aTargetAltitude * 0.95:
        velocity(0, 0, 0, -1, V1D)
        print("Current altitude: {}".format(V1D.location.global_relative_frame.alt))
        time.sleep(0.5)
    velocity(0, 0, 0, 0, V1D)




