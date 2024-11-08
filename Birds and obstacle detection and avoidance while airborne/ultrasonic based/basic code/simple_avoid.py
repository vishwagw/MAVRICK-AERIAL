import time
import gpio # for raspberry pi
from dronekit import Drone 

#GPIO pin setup for ultrasonic Sensor
TRIG_PIN = 23
ECHO_PIN = 24

#constants
# Safe distance in centimeters
SAFE_DISTANCE = 100

# Initialize GPIO
gpio.setmode(gpio.BCM)
gpio.setup(TRIG_PIN, gpio.OUT)
gpio.setup(ECHO_PIN, gpio.OUT)

#initialize drone
drone = Drone()
drone.takeoff()

def get_distance():
    # Trigger the ultrasonic sensor
    gpio.output(TRIG_PIN, True)
    time.sleep(0.00001)
    gpio.output(TRIG_PIN, False)

    # Measure the time it takes for the echo to return
    while gpio.input(ECHO_PIN) == 0:
        pulse_start = time.time()

    while gpio.input(ECHO_PIN) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Calculate distance in cm
    distance = round(distance, 2)

    return distance

def obstacle_avoidance():
    try:
        while True:
            distance = get_distance()
            print(f"Distance to obstacle: {distance} cm")

            if distance < SAFE_DISTANCE:
                print("Obstacle detected! Stopping and adjusting course.")
                drone.stop()  # Stop drone
                time.sleep(1)
                drone.turn_right(30)  # Turn 30 degrees to avoid obstacle
                time.sleep(1)
                drone.forward()  # Resume forward movement
            else:
                drone.forward()  # Keep moving forward if no obstacle
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("Landing and cleaning up.")
        drone.land()
        gpio.cleanup()

#now let's execute the program
obstacle_avoidance()
