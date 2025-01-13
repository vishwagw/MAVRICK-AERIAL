import time
# libs:
import time
import gpio as io
from __future__ import print_function
# connect pymavlink
from pymavlink import mavutil
# connect dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command


# function for the velocity:
# connecting to drone
# ignore if you hacve already connected to drone:
V1D = connect('./', wait_ready=True, baud=921000)

def velocity(velocity_x, velocity_y, yaw_rate, velocity_z, V1D):

    msg = V1D.message_factory.set_position_target_local_ned_encode(0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                                                                   0b0000011111000111, 0, 0, 0, velocity_x, velocity_y,
                                                                   velocity_z, 0, 0, 0, 0, radians(yaw_rate))
    
    V1D.send_mavlink(msg)
    time.sleep(0.1)

