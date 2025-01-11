# Set GPS type (e.g., UBlox)
vehicle.parameters['GPS_TYPE'] = 1  # 1 for UBlox GPS

# Set the GPS baud rate if needed
vehicle.parameters['SERIAL3_BAUD'] = 115  # Match your GPS baud rate

print("GPS configured!")
