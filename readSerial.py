import serial

ser = serial.Serial(port='COM9', baudrate=115200, timeout=0.1)

while True:
    ser_bytes = ser.readline().decode().strip()
    if ser_bytes:  # Skip empty lines
        current_lat, current_long, current_heading, current_pace = map(float, ser_bytes.split(" "))  # Convert to float if necessary
        print(f"Value 1: {current_lat}, Value 2: {current_long}, Value 3: {current_heading}, Value 4: {current_pace}")



