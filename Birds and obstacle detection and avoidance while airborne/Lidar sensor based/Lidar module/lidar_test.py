from lidar import Lidar

lidar_test = Lidar()
connected = lidar_test.connect(1)

if connected < -1:
    print("Not Connected...")

print(lidar_test.getDistance())
print(lidar_test.getVelocity())

