#importing the simulator module
import sim

class Sensor:

    __LEFT_SENSOR_BUSS = 1
    __FRONT_SENSOR_BUSS = 2
    __RIGHT_SENSOR_BUSS = 3
    __BOTTOM_SENSOR_BUSS = 4

    # def __init__(self):
        # When not simulating the data, we will use this class object to get the data from the sensors.
        # self.__left_sensor = LidarLite(self.__LEFT_SENSOR_BUSS)
        # self.__front_sensor = LidarLite(self.__FRONT_SENSOR_BUSS)
        # self.__right_sensor = LidarLite(self.__RIGHT_SENSOR_BUSS)
        # self.__below_sensor = LidarLite(self.__BOTTOM_SENSOR_BUSS)

    #This fucntion establish the connection to the sensors.
    def connect(self):
        # Since I simulate the data within the software there's no need to connect to the sensors,
        # and for this reason this code is under comment and  returns a constant value only to show that this step
        # is needed.
        # conn1 = conn2 = conn3 = conn4 = -2
        # for i in range(0,5):
        #     if conn1<-1:
        #         conn1 = self.__left_sensor.connect()
        #     if conn2<-1:
        #         conn2 = self.__front_sensor.connect()
        #     if conn3<-1:
        #         conn3 = self.__right_sensor.connect()
        #     if conn4<-1:
        #         conn4 = self.__below_sensor.connect()
        #
        #     if conn1<-1 or conn2<-1 or conn3<-1 or conn4<-1:
        #         continue
        #     else:
        #         return 0
        # return -1
        return 0
    
    # Returns the distance measurement from the front server
    def check_ahead():
        # When not simulating the data - next line will be instead in order to get real data from sensor
        # ahead_read = self.__front_sensor.get_distance()
        ahead_read = sim.ahead_reading()
        #the CAUTION DISTANCE from ObstacleAvoidance class.
        if ahead_read <= 0 or ahead_read > 200:
            sim.skip()
        return ahead_read
    
    # Returns distance measurement from the left sensor
    def check_left_side(self):
        # When not simulating the data - next line will be instead in order to get real data from sensor
        # left_side_read = self.__left_sensor.get_distance()
        left_side_read = sim.left_reading()
        return left_side_read
    
    # Returns distance measurement from the right sensor
    def check_right_side(self):
        # When not simulating the data - next line will be instead in order to get real data from sensor
        # right_side_read = self.__right_sensor.get_distance()
        right_side_read = sim.right_reading()
        return right_side_read
    
    #  Returns distance measurament from the sensor below.
    def check_below(self):
        # When not simulating the data - next line will be instead in order to get real data from sensor
        # below_read = self.__below_sensor.get_distance()
        below_read = sim.below_reading()
        return below_read
    
    
