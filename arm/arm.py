from dronekit import connect, VehicleMode
import time

# path for cnnecting the prototype:
connect_port = '/dev/ttyACM0'
baud_rate = 115200

# connection
print(f"Connecting to prototype via: {connect_port}")
V1Drone = connect(connect_port, baud=baud_rate, wait_ready=True, heartbeat_timeout=180)
print("Successfully connected to prototype.")


#ready to arming:
# function:
def arm_drone():
    print("Checking pre-arm status")
    while not V1Drone.is_armable:
        print("pre-arm checked")
        print("Waiting for prototype to to initialize..")
        time.sleep(1) # 1 sec

    print("Arming the motors")
    V1Drone.mode = VehicleMode("GUIDED")
    V1Drone.armed = True

    while not V1Drone.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Prototype has successfully armed..")

# now we must disarm back to stop the motors:
# function:
def disarm_drone():
    print("Disarming the prototype..")
    V1Drone.armed = False
    while V1Drone.armed:
        print("Waiting for disarming...")
        time.sleep(1)

    print("Prototype has successfully disarmed.")

# executing both function as one:
try:
    arm_drone()
    time.sleep(10)
    disarm_drone()
except Exception as e:
    print("Error: arming failed. check the prototype for troubleshooting.")

finally:
    V1Drone.close()
    print("Prototype shutdown.")
    print("mission completed...")

