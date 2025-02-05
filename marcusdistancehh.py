import math
import serial # type: ignore
import time

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

def load_route_coordinates(file_path):
    """
    Load route_coordinates from a text file and handle potential formatting issues.
    
    :param file_path: Path to the text file containing route_coordinates
    :return: List of (latitude, longitude) tuples
    """
    route_coordinates = []
    with open(file_path, 'r') as file:
        for line_number, line in enumerate(file, start=1):
            line = line.strip()
            if not line:
                print(f"Warning: Empty line at {line_number}")
                continue
            
            try:
                lat, lon = map(float, line.split())
                route_coordinates.append((lat, lon))
            except ValueError:
                print(f"Error: Invalid format on line {line_number}: '{line}'")
    
    return route_coordinates

def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the Haversine distance between two GPS points.
    
    :param lat1: Latitude of the first point in decimal degrees
    :param lon1: Longitude of the first point in decimal degrees
    :param lat2: Latitude of the second point in decimal degrees
    :param lon2: Longitude of the second point in decimal degrees
    :return: Distance in meters between the two points
    """
    R = 6371.0  # Radius of the Earth in kilometers
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) *
         math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance * 1000 # convert to meters

route_file = 'MDR.TXT'
route_coordinates = load_route_coordinates(route_file)
ser = serial.Serial(port='COM9', baudrate=115200, timeout=0.1)
waypoint = 0
waypoint_tolerance = 2.5 # meters
prev_lat = 0
prev_long = 0

i = 1
total = 0
start_walk = 0
end_walk = 0

isTiming = False

while True:
    # ----- Read current gnss data -----
    ser_bytes = ser.readline().decode().strip()
    if ser_bytes: # skip empty lines from serial
        gnss_data = ser_bytes.split(" ")
        fusion_status = gnss_data[0]  # Extract as string
        gnss_lat, gnss_long, gnss_heading, gnss_speed = map(float, gnss_data[1:])  # Parse the rest as floats
        
        

        distanceToNextWaypoint = haversine_distance(gnss_lat, gnss_long, route_coordinates[waypoint][0], route_coordinates[waypoint][1])
        headingToNextWaypoint = calculate_bearing(gnss_lat, gnss_long, route_coordinates[waypoint][0], route_coordinates[waypoint][1])
        #print(f"\n\nreported gnss data: {fusion_status} {gnss_lat} {gnss_long} {gnss_heading}{chr(176)} {gnss_speed:.2f}m/s")
        total = total + gnss_speed

        if gnss_speed != 0:
            i = i + 1

        if gnss_speed != 0 and not isTiming:
            start_walk = time.time()
            isTiming = True
        if gnss_speed == 0 and isTiming:
            end_walk = time.time()

        print(f"gnss reported speed: {gnss_speed:.2f}m/s    avg of reported speeds: {total/i:.2f}m/s")
        if end_walk != 0:
            print(f"elapsed time: {end_walk - start_walk:.2f} seconds")
            exit()

        if distanceToNextWaypoint <= waypoint_tolerance:
            # reached waypoint
            print(f"Reached waypoint #{waypoint+1}")
            waypoint = waypoint + 1 # look at next waypoint
        #else:
            # directions to waypoint
            #print(f"To waypoint #{waypoint+1},{route_coordinates[waypoint]} --> Distance: {distanceToNextWaypoint:.2f}m")#, Required heading: {headingToNextWaypoint:.1f}{chr(176)}")