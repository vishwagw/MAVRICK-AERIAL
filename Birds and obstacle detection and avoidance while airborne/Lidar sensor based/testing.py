from obstale_avoid import ObstacleAvoidance
import time

obstacle_avoidance_system = ObstacleAvoidance()
obstacle_avoidance_system.start()

flag = True

while flag:
    time.sleep(0.1)
    if obstacle_avoidance_system.take_control():
        print("OVerride...")

    if obstacle_avoidance_system.is_alive() is False:
        flag = False
        print("Terminated")

