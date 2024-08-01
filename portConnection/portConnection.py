import serial

# Open COM21 to read data from Mission Planner
ser_in = serial.Serial('COM21', 9600, timeout=1)

# Open COM13 to communicate with Arduino
ser_out = serial.Serial('COM13', 9600, timeout=1)

while True:
    if ser_in.in_waiting > 0:
        # Read line from COM21 (Mission Planner)
        line = ser_in.readline().decode('utf-8').strip()
        
        if line.startswith('$GPGGA'):
            print(f"sending GPGGA data: {line}")
            ser_out.write((line + '\n').encode('utf-8'))

    if ser_out.in_waiting > 0:
        # Read response from Arduino
        response = ser_out.readline().decode('utf-8')
        print(f"Received from Arduino: {response.strip()}")  # For debugging