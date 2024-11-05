# need simulator module and vehicle module
from vehicle_main import Vehicle
import sim

class FlightCommands:
    TIME_TO_COLLISON = 10

    # This module should have this constructor which gets a vehicle object in order to send the proper orders
    # to the drone. Since I can't initialize this object, This section and all vehicle-related commands
    # are under comment.

    # def __init__(self, vehicle):
    #     # Check for correct input
    #     if isinstance(vehicle, Vehicle) is False:
    #         raise TypeError('Expected object of type Vehicle, got '+type(vehicle).__name__)
    #
    #     self.__vehicle = vehicle

    #ordering the drone to land:
    def land(self):
        #self.__vehicle_main.landing()
        return
    
    #ordering the drone to maintain altitude:
    def maintain_alt(self):
        # self.__vehicle_main.keep_altitude()
        return
    
    #ordering the drone to fly to right side:
    def go_right(self):
        sim.change_longitude_latitude()
        # self.__vehicle_main.move_right()
        return
    
    #ordering the drone to ascend:
    def go_up(self):
        sim.go_up()
        # self.__vehicle_main.move_up()
        return
    
    #ordering the drone to decend:
    def go_down(self):
        sim.go_down()
        return
    
    # Ordering the drone to slow down in order to give it enough time to perform avoidance maneuvers.
    def slow_down(self, distance):
        if isinstance(distance, float) is False and isinstance(distance,int) is False:
            raise TypeError ('Expected variable of type loat and got a variable of type ' + type(distance).__name__)
        elif distance <= 0:
            raise ValueError('Illegal value. Cannot be 0')
        
        #  # Calculating a new velocity for the drone to give it 10 seconds before colliding. 10 Seconds should be
        #  # a sufficient amount of time to avoid colliding with the detected object.
        #
        # velocity = distance/self.TIME_TO_COLLISION
        # self.__vehicle.set_groundspeed(velocity)
        return
    
    # Order the drone to change its destination to its dock.
    def go_back_to_base(self):
        print('Destination changed: Going Back home')
        # self.__vehicle.get_back_to_station()

    