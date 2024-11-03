#importing libraries
import random
import time

#at the round level:
start_time = round(time.time())
_below_reaing = 1000
_height = 1000
_latitude = 0
_longitude = 0

#then:
try:
    _leftSensorData = open('Sensors Data\\leftSensorData.txt', 'r')
    _middleSensorData = open('Sensors Data\\middleSensorData.txt', 'r')
    _rightSensorData = open('Sensors Data\\rightSensorData.txt', 'r')
except:
    print('ERROR: Can not open sensor data file...')
    
#return front sensor simulated meaurement
def ahead_reading():
    time.sleep(0.1)
    return int(_middleSensorData.readline())


#return left sensor simulated measurements:
def right_reading():
    return int(_rightSensorData.readline())

#return below sensor simulated measurements
def below_reading():
    global _below_reaing
    return _below_reaing

#return the simulated height of the drone given by the pixhawk/ardupilot 
def height_reading():
    global _height
    return _height

#return the simulated latitude position of the drone given by the pixhawk flight controlle
def latitude_reading():
    global _latitude
    return _latitude

#return the simulated longitude position of the drone 
def longitude_reading():
    global _longitude
    return _longitude

#now drone is being told to mov left and right.
#themovements can change the simulated values of latitudes and longitudes
def change_longitude_latitude():
    global _latitude
    global _longitude
    _longitude += random.uniform(1, 3) * 0.00000001
    _latitude += random.uniform(1, 3) * 0.00000004


#front sensor data will indicate that there is no obstacle in the front
#so, simulation will need to skip the 
def go_up():
    global _below_reaing
    global _height
    _height += int(random.uniform(10, 25))
    _below_reaing += int(random.uniform(10, 25))
    return _below_reaing

#when the drone is ordered to go down/descend:
#this method decreases the simulated values of the height of the drone
def go_down():
    global _below_reaing
    global _height
    _height -= int(random.uniform(10, 25))
    _below_reaing -= int(random.uniform(10, 25))

#chenges of the altitude
#changing the height of the drone hoovering or operating
#a heigh difference for more realistic simulation
def altitude_change():
    global _below_reaing
    global _height
    _below_reaing += int(random.uniform(-15, 15))
    _height = _below_reaing + int(random.uniform(0, 23))

    