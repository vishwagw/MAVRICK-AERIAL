#libraries
import math

#import simulator.py
import sim

class FlghtData:

    __EARTH_RADIUS_IN_METERS = 6371000

    # This module should have this constructor which gets a vehicle object in order to send the proper orders
    # to the drone. Since I can't initialize this object, This section and all vehicle-related commands
    # are under comment.

    # def __init__(self, vehicle):
    #     # Check for correct input
    #     if isinstance(vehicle, Vehicle) is False:
    #         raise TypeError('Expected object of type Vehicle, got '+type(vehicle).__name__)
    #
    #     self.__vehicle = vehicle
    #     return

    # Returns the distance between 2 GPS coordiantes represented by latitude-longitude parameters
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        if not isinstance(lat1, float) or not isinstance(lon1, float) or not isinstance(lat2, float) or not isinstance(lon2, float):
            raise TypeError('Expected type float..')
        
        phi1 = self.__degree_to_radian(lat1)
        phi2 = self.__degree_to_radian(lat2)
        dPhi1 = self.__degree_to_radian((lat2 - lat1))
        dLambda = self.__degree_to_radian((lon2 - lon1))

        a = math.sin(dPhi1/2)*math.sin(dPhi1/2) + math.cos(phi1)*math.cos(phi2)*math.sin(dLambda/2)*math.sin(dLambda/2)
        c = 2* math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance_in_metres = self.__EARTH_RADIUS_IN_METERS*c
        return distance_in_metres
    
    #Return te current latitude of the drone according to its GPS device
    def get_current_latitude(self):
        # return self.__vehicle_main.get_location_latitude()
        return float(sim.latitude_reading())
    
    # Return te current longitude of the drone according to its GPS device.
    def get_current_longitude(self):
        # return self.__vehicle_main.get_location_longitude()
        return float(sim.longitude_reading())
    
    # Reading the current height of the drone according to its devices in centimeters
    def get_current_height(self):
        # return self.__vehicle_main.get_location_altitude()
        return int(sim.height_reading())
    
    
