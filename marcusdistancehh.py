import math
import serial

def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate the bearing between two points on the earth (specified in decimal degrees).
    
    :param lat1: Latitude of the starting point
    :param lon1: Longitude of the starting point
    :param lat2: Latitude of the destination point
    :param lon2: Longitude of the destination point
    :return: Bearing in degrees from the starting point to the destination
    """
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)
    
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    initial_bearing = math.atan2(x, y)
    compass_bearing = (math.degrees(initial_bearing) + 360) % 360
    
    return compass_bearing

def determine_direction(current_bearing, target_bearing):
    """
    Determine the direction to turn to reach the target bearing from the current bearing.
    
    :param current_bearing: The current heading in degrees
    :param target_bearing: The target heading in degrees
    :return: Instruction on which direction to take
    """
    difference = (target_bearing - current_bearing + 360) % 360

    if difference == 0:
        return "Go straight"
    elif difference > 180:
        turn_angle = 360 - difference
        return f"Turn left {turn_angle:.1f} degrees"
    else:
        turn_angle = difference
        return f"Turn right {turn_angle:.1f} degrees"

def load_coordinates(file_path):
    """
    Load coordinates from a text file and handle potential formatting issues.
    
    :param file_path: Path to the text file containing coordinates
    :return: List of (latitude, longitude) tuples
    """
    coordinates = []
    with open('marcus_loop.TXT', 'r') as file:
        for line_number, line in enumerate(file, start=1):
            line = line.strip()
            if not line:
                print(f"Warning: Empty line at {line_number}")
                continue
            
            try:
                lat, lon = map(float, line.split())
                coordinates.append((lat, lon))
            except ValueError:
                print(f"Error: Invalid format on line {line_number}: '{line}'")
    
    return coordinates

def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the Haversine distance between two GPS points.
    
    :param lat1: Latitude of the first point in decimal degrees
    :param lon1: Longitude of the first point in decimal degrees
    :param lat2: Latitude of the second point in decimal degrees
    :param lon2: Longitude of the second point in decimal degrees
    :return: Distance in kilometers between the two points
    """
    R = 6371.0  # Radius of the Earth in kilometers
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) *
         math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance


# Load coordinates from file and generate directions
file_path = 'marcus_loop.TXT'  # Path to your file
coordinates = load_coordinates(file_path)

ser = serial.Serial(port='COM9', baudrate=115200, timeout=0.1)
i = 0

while True:
    # Read current gnss data
    ser_bytes = ser.readline().decode().strip()
    if ser_bytes: # new data
        current_lat, current_long, current_heading, current_pace = map(float, ser_bytes.split(" "))
        print(f"\ngnss data: {current_lat} {current_long} {current_heading}")# {current_pace}")
        
        lat_diff = abs(current_lat - coordinates[i][0])
        long_diff = abs(current_long - coordinates[i][1])
        tolerance = 0.00001
        print(f"lat_diff: {lat_diff:.7}, long_diff: {long_diff:.7}")
        if lat_diff < tolerance and long_diff < tolerance:
            #reached waypoint
            i = i + 1 # look at next waypoint
            print(f"Reached waypoint #{i}")
        else:
            #get distance to next waypoint from current posistion
            distanceToNext = haversine_distance(current_lat, current_long, coordinates[i][0], coordinates[i][1])
            #get direction to next wapoint
            headingToNextWaypoint = calculate_bearing(current_lat, current_long, coordinates[i][0], coordinates[i][1])
            directionToNext = determine_direction(current_heading, headingToNextWaypoint)
            print(f"To waypoint {i}: Distance {distanceToNext:.7f}, {directionToNext}\n\n")

