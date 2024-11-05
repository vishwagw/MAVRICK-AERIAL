#libraries 
import math
import time
import sys
import socket
#drone libraries
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
#command message definitions
from pymavlink import mavutil

#create the class-vehicle 
class Vehicle:
    #building constructor
    def __init__(self, connectionString):
        print('Connecting to vehicle on : %s' % connectionString)
        self.vehicle = connect(connectionString, baud=57600)
        print('Connected!\n')
        
    #now arm the vehicle and takeoff
    def arm_and_takeoff(self, takeOffAltitude):

        #sleep for 1sec
        time.slee()
        # vehicle arming start
        self.vehicle_arm()
        #checking wheter vehicle is arming
        self.vehicle_arming_check()
        #sleep for 1sec
        time.sleep()
        #arming and takeoff function activate:
        self.arm_and_takeoff(takeOffAltitude)

    #setting parameters and getting the parameters:
    def set_parameters(self, parameter, value):
        self.vehicle.parameters[parameter] = value

    def get_paramteres(self, parameter):
        value = self.vehicle.parameters[parameter]
        print("Param: %s" % self.vehicle.parameters[parameter])
        return value
    
    #lets add any parameter callback function
    def any_parameter_callback(self, attr_name, value):
        print(" ANY PARAMETER CALLBACK: %s changed to: %s" % (attr_name, value))

    #just adding an observer 
    def  set_on_parameter_changed_listener(self):
        self.vehicle.parameters.add_attribute_listener('*', self.any_parameter_callback)

    #set channel
    def set_chnl(self, channel_num, value):
        self.vehicle.mode = VehicleMode('STABALIZERS')
        channel_value = self.vehicle.channels[str(channel_num)]

        if (channel_value + value) < 0:
            print("<<<<<<<<< value too low")
            return
        
        print("===================================>channel: " + str(channel_num) + "      " + str(channel_value + value))
        print("==>> " , channel_value)
        self.vehicle.channels.overrides[str(channel_num)] = (channel_value + value)
        #self.vehicle.channel_override = {str(channel_num) : (channel_value + value)}
        self.vehicle.flush()
        print("===> " , self.vehicle.channels)
        
    #channel information
    def get_chanl_info(self):
         
        # getting channel values
        print("Channel values from RC Tx:", self.vehicle.channels)
        print("Number of channels: %s" % len(self.vehicle.channels))

        #overiding the channel
        print("\nChannel overrides: %s" % self.vehicle.channels.overrides)

    #setting parameters 
    def air_speed(self, speed):
        self.vehicle.airspeed = speed

    def rtlMood(self):
        self.vehicle.mode = VehicleMode('RTL')

    def guidedMode(self):
        self.vehicle.mode = VehicleMode('GUIDED')

    def cloesVec(self):
        self.vehicle.close()

    def set_mode(self, mode):
        self.vehicle.mode = mode

    def set_armed(self, bool):
        self.vehicle.armed = bool
    
    def set_groundspeed(self, speed):
        self.vehicle.groundspeed = speed
        return

    def set_heading(self, heading):
        self.vehicle_condition_yaw(heading)
        return
    

    #getting all the parameters 
    def get_location(self):
        return self.vehicle.location.global_frame
    
    def get_location_latitude(self):
        return self.vehicle.location.lat
    
    def get_location_longitude(self):
        return self.vehicle.location.long
    
    def get_location_altitude(self) :
        return self.vehicle.location.alt 
    
    def get_location_global_frame(self):
        return self.vehicle.location.global_frame
    
    def get_location_global_relative(self):
        return self.vehicle.location.global_relative_frame
    
    def get_location_local_frame(self):
        return self.vehicle.location.local_frame

    def get_attitude(self):
        return self.vehicle.attitude
    
    def get_velocity(self):
        return self.vehicle.velocity

    def get_gps(self):
        return self.vehicle.gps_0

    def get_heading(self):
        return self.vehicle.heading

    def is_armable(self):
        return self.vehicle.is_armable

    def get_system_status(self):
        return self.vehicle.system_status.state

    def get_groundspeed(self):
        return self.vehicle.groundspeed

    def get_airspeed(self):
        return self.vehicle.airspeed

    def get_mode(self):
        return self.vehicle.mode.name

    def get_home_location(self):
        return self.vehicle.home_location

    def get_battery_voltage(self):
        return self.vehicle.battery.voltage

    def get_battery_current(self):
        return self.vehicle.battery.current

    def get_battery_level(self):
        if self.vehicle.battery.level is None:
            return -1
        return self.vehicle.battery.level

    def get_distance_metres(aLocation1, aLocation2):
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
    
    ### let's now print the following data / parameters

    def print_vehicle_state(self):
        print('|  Pitch: $%.2f  |  Yaw: $%.2f  |  Roll: $%.2f  |  Heading: $%.2f  |' % (self.vehicle.attitude.pitch, self.vehicle.attitude.yaw, self.vehicle.attitude.roll, self.vehicle.heading))

    ### get and print all the attributes for drone state
    def print_vehicle_full_state_info(self):
        #printing all the followin attributes of drone state:

        print("\nGet all vehicle attribute values:")
        print(" Global Location: ...................... %s" % self.vehicle.location.global_frame)
        print(" Global Location (relative altitude): .. %s" % self.vehicle.location.global_relative_frame)
        print(" Local Location: ....................... %s" % self.vehicle.location.local_frame)
        print(" Attitude: ............................. %s" % self.vehicle.attitude)
        print(" Velocity: ............................. %s" % self.vehicle.velocity)
        print(" GPS: .................................. %s" % self.vehicle.gps_0)
        print(" Gimbal status: ........................ %s" % self.vehicle.gimbal)
        print(" Battery: .............................. %s" % self.vehicle.battery)
        print(" EKF OK?: .............................. %s" % self.vehicle.ekf_ok)
        print(" Last Heartbeat: ....................... %s" % self.vehicle.last_heartbeat)
        print(" Rangefinder: .......................... %s" % self.vehicle.rangefinder)
        print(" Rangefinder distance: ................. %s" % self.vehicle.rangefinder.distance)
        print(" Rangefinder voltage: .................. %s" % self.vehicle.rangefinder.voltage)
        print(" Heading: .............................. %s" % self.vehicle.heading)
        print(" Is Armable?: .......................... %s" % self.vehicle.is_armable)
        print(" System status: ........................ %s" % self.vehicle.system_status.state)
        print(" Groundspeed: .......................... %s" % self.vehicle.groundspeed)  # settable
        print(" Airspeed: ............................. %s" % self.vehicle.airspeed)  # settable
        print(" Mode: ................................. %s" % self.vehicle.mode.name)  # settable
        print(" Armed: ................................ %s" % self.vehicle.armed)  # settable
        print("\n \n")

    
    #now we have developed the basic parameters 
    # we also haved developed the arm and takeoff function

    ### vehicle states and movements
    def move_up(self):
        print('Moving Up')
    
    def move_down(self):
        print("moving down")

    def move_left(self):
        print("moving left")

    def move_right(self):
        print("moving right")

    def move_foreword(self):
        print("moving foreword")

    def move_backward(self):
        print("moving backward")

    def stop_moving(self):
        print("stop moving")

    def landing(self):
        print("landing")

    def keep_altitude(self):
        print("keeping altitude")

    def get_back_to_station(self):
        print("Going back to station")

    ### loop until the drone is initialize

    def vehicle_pre_arm_check(self):
        while not self.vehicle.is_armable:
            print("Waiting for Autonomous Dron to initialize...")
            time.sleep(1)


    ### motor control
    # starting to arm the motor:
    def vehicle_arm(self) :
        print('==> Vehicle start arming...')
        #UAV should be armed in GUIDED mode:
        self.vehicle.mode = VehicleMode('GUIDED')
        self.vehicle.armed = True

    # system on a Loop until motors are armed:
    def vehicle_arming_check(self):
        while not self.vehicle.armed:
            print('==> Waiting for vehicle to arm')
            time.sleep(1)
            self.vehicle_arm()
            # self.vehicle_armed = True
        print('\n==> Vehicle ARMED!\n')

    #Taking off the drone to the given altitude 
    # system is in the Loop until the the drone reaches the altitude
    def simpleGoTo(self, lat, lng, alt, velocity=-1):
        # dest = LocationGlobalRelative(lan, lng, height)
        # dest - destination
        # wil be set to default travel speed
        if velocity == -1:
            self.vehicle.airspeed = 1
        else:
            self.vehicle.airspeed = velocity

        dest = LocationGlobalRelative(lat, lng, alt)
        self.vehicle.simple_goto(dest)

    #setting vehicle mode to RTL
    def vehicle_RTL(self):
        self.vehicle.mode = VehicleMode('RTL')

    #now we are taking the drone to a given location
    def vehicle_goto_location(self, location):
        currentLocation = self.vehicle.location
        targetDistance = self.get_distance_metres(currentLocation, location)
        self.gotoFunction(location)
        self.vehicle.flush()
        # Stoping the action if UAV is no longer in guided mode.
        while not self.api.exit and self.vehicle.mode.name == 'GUIDED':
            remainingDistance = self.get_distance_metres(self.vehicle.location, location)
            # Just below target, in case of undershoot.
            if remainingDistance <= targetDistance * 0.01:
                print("Reached Target.")
                break
            time.sleep(2)

    #take the UAV to a speified location
    def simpleGoTo(self, lat, lng, alt, velocity=-1):
        # dest = LocationGlobalRelative(lat, lon, height)
        # set the default travel speed
        if velocity == -1:
            self.vehicle.airspeed = 1
        else:
            self.vehicle.airspeed = velocity

        dest = LocationGlobalRelative(lat, lng, alt)
        self.vehicle.simple_goto(dest)

    #setting UAV mode to RTL
    def vehicle_RTL(self):
        self.vehicle.mode = VehicleMode('RTL')
        self.vehicle.flush()

    #take the UAV to givem location
    def vehicle_goto_location(self,location):
        currentLocation = self.vehicle.location
        targrtDistance = self.get_distance_metres(currentLocation, location)
        self.gotoFunction(location)
        self.vehicle.flush()
        # to stop the actions if UAV is not in GUIDED mode anymore
        while not self.api.exit and self.vehicle.mode.name == 'GUIDED':
            remainingDistance = self.get_distance_metres(self.vehicle.location, location)
            if remainingDistance <= targrtDistance *0.01:
                print ("UAV has reached the TARGET..")
                break
            
    #check the README.md -> MAV_CMD_CONDITION_YAW

    def vehicle_condition_yaw(self, heading, relative=False):
        if relative:
            # yaw relative to direction of travel
            is_relative = 1
        else:
            #yaw is an absolute angel
            is_relative = 0
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            heading,
            0,
            1,
            is_relative,
            0, 0, 0)
        
        # send the command to UAV
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    
    def vehicle_rotate_camera_gimbal(self, location) :
        #create the MAV_CMD_DO_SET_ROI command
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,
            0,
            0, 0, 0, 0,
            location.lat,
            location.long,
            location.alt
        )

        #send the command to UAV
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    # DISTANCE AND UAV
    # refer README.md -> returns locationglobal

    def get_location_metres(original_location, dNorth, dEast):

        #raius of planet earth
        # Radius of "spherical" earth
        # Coordinate offsets in radians
        earth_radius = 6378137.0
        dLat = dNorth / earth_radius
        dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

        #new location position in degre
        newlat = original_location.lat + (dLat * 180 / math.pi)
        newlon = original_location.lon + (dLon * 180 / math.pi)
        if type(original_location) is LocationGlobal:
            targetlocation = LocationGlobal(newlat, newlon, original_location.alt)
        elif type(original_location) is LocationGlobalRelative:
            targetlocation = LocationGlobalRelative(newlat, newlon, original_location.alt)
        else:
            raise Exception('INVALID location object passed')
        
        return targetlocation
    
    # Returns the ground distance in metres between two LocationGlobal objects.

    def get_distance_metres(alocation1, alocation2):

        dlat = alocation2.lat - alocation1.lat 
        dlong = alocation2.lon - alocation1.lon 
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
    
    # Returns the bearing between the two LocationGlobal objects passed as parameters
    def get_bearing(alocation1, alocation2):
        off_x = alocation2.lon - alocation1.lon 
        off_y = alocation2.lat - alocation1.lon 
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing
    
    ## MOVING THE UAV ##

    #refer README.md -> send_ned velocity
    def send_ned_velocity(self,velocity_x, velocity_y, velocity_z):

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0,
            velocity_x, velocity_y, velocity_z,
            0, 0, 0,
            0, 0
        )

        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def send_global_velocity(self, velocity_x, velocity_y, velocity_z):
        
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0, 
            0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111000111, 
            0,
            0,
            0,
            velocity_x,
            velocity_y,
            velocity_z,
            0, 0, 0,
            0, 0
        )

        # send command to vehicle on 1 Hz cycle
        self.vehicle.send_mavlink(msg)
        
    