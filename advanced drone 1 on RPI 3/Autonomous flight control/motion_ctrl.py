#importing libraries 
import numpy as np
import cv2
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, Vehicle, LocationGlobal
import time
import math

#connecting the vehicle 
V1drone = connect("udp:192.168.137.103:14550", wait_ready=True)

#video:
cap = cv2.VideoCapture(0)

#function for arming the drone and takeoff
#taking off drone to  target altitude
def arm_and_takeoff(aTargetAltitude):
    print("Pre-checking the drone..")
    #do not arm  drone until autopilot is ready
    while not V1drone.is_armable:
        print("Waiting for V1 drone vehicle to initialize...")
        time.sleep(1)

    print("Checking motors..")
    print("Arming motorw...")
    # vehicle should arm in DUIDED MODE.
    V1drone.mode = VehicleMode("GUIDED..")
    V1drone.armed = True

    time.sleep(2)

    print(V1drone.mode)

    # confirm vehicle armed before attemting to take off
    while not V1drone.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("V1drone Taking-off")
    # Take off to target altitude
    V1drone.simple_takeoff(aTargetAltitude)
    # Wait until the vehicle reaches a safe height before processing the goto mode.
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately)
    while True:
        print(" Altitude: ", V1drone.location.global_relative_frame.alt )
        # Break and return from function just below target altitude.
        if V1drone.location.global_relative_frame.alt >= aTargetAltitude*0.9:
            print(" V1drone have Reached Target altitude...")
            break
        time.sleep(1)

#initializing the above function-
arm_and_takeoff(10)

#altitude calibrating
degree = 1
east = 0
north = 0 

#next function is for the location of the drone.
# d- drone
def get_location_metres(original_loc, dNorth, dEast):

    earth_radius = 6378137.0 #Earth's radius is approximately 6,371 kilometers
    #coordinating the offsets in radius
    dlat = dNorth/earth_radius
    dlong = dEast/(earth_radius*math.cos(math.pi*original_loc.lat/180))

    #new position in decimal degrees
    newlat = original_loc.lat + (dlat * 180/math.pi)
    newlong = original_loc.lon + (dlong * 180/math.pi)

    if type(original_loc) is LocationGlobal:
        targetlocation = LocationGlobal(newlat, newlong, original_loc.alt)
    elif type(original_loc) is LocationGlobalRelative:
        targetlocation = LocationGlobalRelative(newlat, newlong, original_loc.alt)
    else:
        raise Exception("Invalid Location object passed")
    
    return targetlocation;

#function for getting the distance metres->
def get_distance_metres(alocation1, alocation2):

    dlat = alocation2.lat - alocation1.lat
    dlong = alocation2.long - alocation1.long
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

#goto function initializing. 
def goto(dNorth, dEast, gotoFunction=V1drone.simple_goto):
    currentLocation = V1drone.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)

#yaw control function
def condition_yaw(heading, relative=False):

    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    #creating the CONDITION_YAW command using command_long_encode()
    msg = V1drone.message_factory.command_long_encode(
        0, 0, # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading, #param 1, yaw in degrees
        0, #param 2, yaw in degree/s
        1, # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0) # param 5 - 7 not used
    #sending the command to the vehicle:
    V1drone.send_mavlink(msg)

northArray = []
eastArray = []

while True:
    ret, frame = cap.read()
    #frame = cv2.flip(frame, -1)
    if ret == True:
        # Filter red color
        # frame = cv2.bilateralFilter(frame,9,75,75)
        frame_hsv =cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(frame_hsv, (0, 70, 50), (10, 255, 255))
        mask2 = cv2.inRange(frame_hsv, (170, 70, 50), (180, 255, 255))
        mask = mask1 + mask2
        # Edge detection
        canny_output = cv2.Canny(mask, 10, 10, 20)
        # Find the center
        pixels = np.where(canny_output == 255)
        cX = np.average(pixels[1])
        cY = np.average(pixels[0])

        # cX cY -> NaN
        cv2.circle(canny_output, (int(cX),int(cY)), 3, (255,255,255), thickness=10, lineType=8, shift=0)

        x = cX-320
        y = 240-cY
        Rsquare = math.sqrt(abs(x)**2 + abs(y)**2)/10
        print(math.degrees(math.atan(y/x)))
        degree = math.degrees(math.atan(y/x))
        print(degree, east, north, "3")
        if x <0 and y<0: # 
            degree = 27 - degree
            print(degree,east,north,"3")


        elif (x<0 and y>0): #II.Region
            degree = abs(degree-270)
            print(degree, east, north,'2')

        elif x>0 and y < 0:
            degree = 90-degree
            print(degree,east,north,"4")

        elif x>0 and y >0:
            degree = 90-degree
            print(degree,east,north,"1")

        east = math.sin(math.radians(degree))*Rsquare
        north = math.cos(math.radians(degree))*Rsquare

        #initializing the goto function:
        goto(north, east)

        condition_yaw(degree, False)
        cv2.imshow("contours", canny_output)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()


