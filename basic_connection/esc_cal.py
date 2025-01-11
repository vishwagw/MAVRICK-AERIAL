# Arm the vehicle
vehicle.armed = True

# Start ESC calibration
print("Starting ESC calibration...")
vehicle.parameters['ESC_CALIBRATION'] = 1

# Wait for the calibration process
input("Disconnect the power and reconnect to complete calibration. Press Enter when done...")
