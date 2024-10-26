import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from __future__ import print_function
from pymavlink import mavutil
from gpiozero import LED, Button

V1drone = connect('/dev/serial0', wait_ready=True, baud=921000)

#creating the take off finction :
def arm_and_takeoff(aTargetAltitude) :
    while not V1drone.is_armable:
        time.sleep(1)

    V1drone.mode = VehicleMode("GUIDED")
    V1drone.armed = True

    # Confirm vehicle armed before attempting to take off
    while not V1drone.armed:
        time.sleep(1)

    V1drone.simple_takeoff(aTargetAltitude)

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        #break and return from function just below target altitude.
        if V1drone.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            break
        time.sleep(1)

#initializing the function
arm_and_takeoff(1)

