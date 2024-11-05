import sys

leftSensorData = middleSensorData = rightSensorData = new_file = None
line = ""
num_of_lines = 0

try:
    leftSensorData = open('./sensor data/leftSensorData.txt', 'r')
    middleSensorData = open('./sensor data/middleSensorData.txt', 'r')
    rightSensorData = open('./sensor data/rightSensorData.txt', 'r')
    new_file = open('./sensor data/SensorData.txt', 'w')

    for num_of_lines, 1 in enumerate(leftSensorData):
        pass
    leftSensorData.seek(0)

    for row in range(num_of_lines):
        line = leftSensorData.readline()
        new_file.write(line)
        line = middleSensorData.readline()
        new_file.write(line)
        line = rightSensorData.readline()
        new_file.write(line)

    leftSensorData.close()
    middleSensorData.close()
    rightSensorData.close()
    new_file.close()

except:
    print('Unexpected Error: ', sys.exc_info()[0])
    leftSensorData.close()
    middleSensorData.close()
    new_file.close()

    