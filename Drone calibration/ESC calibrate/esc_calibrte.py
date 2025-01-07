from dronekit import connect, VehicleMode
import time

# Connect to the flight controller
connection_string = "/dev/ttyACM0"  # Replace with your connection string
baud_rate = 57600  # Standard baud rate for Pixhawk
print(f"Connecting to vehicle on {connection_string}")
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)

def calibrate_esc():
    try:
        print("Starting ESC Calibration...")

        # Step 1: Set vehicle to manual mode
        print("Setting vehicle to STABILIZE mode")
        vehicle.mode = VehicleMode("STABILIZE")
        time.sleep(2)

        # Step 2: Arm the vehicle
        print("Arming vehicle...")
        vehicle.armed = True
        time.sleep(2)

        # Step 3: Send max throttle
        print("Sending max throttle...")
        vehicle.channels.overrides = {'3': 2000}  # Channel 3 is throttle
        time.sleep(4)

        # Step 4: Disarm vehicle to initialize ESC calibration
        print("Disarming vehicle...")
        vehicle.armed = False
        time.sleep(2)

        # Step 5: Send min throttle
        print("Sending min throttle...")
        vehicle.channels.overrides = {'3': 1000}
        time.sleep(4)

        # Step 6: Complete calibration
        print("ESC Calibration Complete.")
        vehicle.channels.overrides = {}

    except Exception as e:
        print(f"Error during ESC calibration: {e}")
    finally:
        vehicle.close()

# Run the ESC calibration
calibrate_esc()
