import numpy as np
import cv2
#from picamera import PiCamera
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, Vehicle, LocationGlobal, Command
import time
from time import gmtime, strftime

#connect vehicle
V1drone = connect("/dev/serial0", wait_ready=True, baud=921000)

#function for takeoff and arm-
def arm_and_takeoff(aTargetAltitude) :
    print("Basic pre-arm checks")
    while not V1drone.is_armable:
        print("Waiting for V1drone to initialize..")
        time.sleep(1)
    
    print("Arming motors")
    V1drone.mode = VehicleMode("GUIDED")
    V1drone.armed = True

    time.sleep(2)

    print(V1drone.mode)

    # Confirm vehicle armed before attempting to take off
    while not V1drone.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    V1drone.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto function
    while True:
        print(" Altitude: ", V1drone.location.global_relative_frame.alt)
        #break and return from the function just below target altitude
        if V1drone.location.global_relative_frame.alt >= aTargetAltitude*0.90:
            print("V1drone has reached the target altitiude...")
            break
        time.sleep(1)

def first_tour():
    cmds = V1drone.commands
    cmds.clear()
    V1drone.flush()
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 5))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6940856 ,35.4609275 , 5))#2
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6941850 ,35.4606901 , 5))#2
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6941850 ,35.4606901 , 5))#2

    print("Upload the new command to V1drone..")
    cmds.upload()

arm_and_takeoff(5)
first_tour()
V1drone.mode = VehicleMode("AUTO")
V1drone.commands.next= 0

while V1drone.commands.next <= 2:
    nextwaypoint = V1drone.commands.next

