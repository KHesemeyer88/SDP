import math


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

    if difference < 10 or difference > 350:
        return "Go straight"
    elif difference > 180:
        return "Turn left"
    elif difference < 180:
        return "Turn right"
    else:
        return "U-turn"

def load_coordinates(file_path):
    """
    Load coordinates from a text file and handle potential formatting issues.
    
    :param file_path: Path to the text file containing coordinates
    :return: List of (latitude, longitude) tuples
    """
    coordinates = []
    with open('marcus_loop.txt', 'r') as file:
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

def determine_direction(current_bearing, target_bearing):
    """
    Determine the direction to turn to reach the target bearing from the current bearing, 
    including the exact angle of the turn.
    
    :param current_bearing: The current heading in degrees
    :param target_bearing: The target heading in degrees
    :return: Instruction on which direction to take with the angle in degrees
    """
    difference = (target_bearing - current_bearing + 360) % 360

    if difference == 0:
        return "Go straight"
    elif difference > 180:
        turn_angle = 360 - difference
        return f"Turn left {turn_angle:.1f} degrees"
    else:
        return f"Turn right {difference:.1f} degrees"

def generate_directions_and_distance(coordinates):
    """
    Generate directional instructions and calculate total travel distance based on GPS coordinates.
    
    :param coordinates: List of (latitude, longitude) tuples
    :return: List of directions after each measurement and total distance traveled
    """
    directions = []
    step_distances = []
    total_distance = 0.0

    if len(coordinates) < 2:
        return directions, step_distances, total_distance

    current_bearing = 0  # Assume initial heading north

    for i in range(1, len(coordinates)):
        # Calculate the target bearing from the current point to the next
        target_bearing = calculate_bearing(
            coordinates[i-1][0], coordinates[i-1][1],
            coordinates[i][0], coordinates[i][1]
        )
        
        # Determine direction based on current and target bearing
        direction = determine_direction(current_bearing, target_bearing)
        
        # Calculate distance between consecutive coordinates
        distance = haversine_distance(
            coordinates[i-1][0], coordinates[i-1][1],
            coordinates[i][0], coordinates[i][1]
        )
        
        # Add distance to the total and record the step distance
        total_distance += distance
        step_distances.append(distance)
        
        # Append direction and distance info
        directions.append((coordinates[i], direction, distance))
        
        # Update current bearing for the next iteration
        current_bearing = target_bearing
    
    return directions, step_distances, total_distance


# Load coordinates from file and generate directions
file_path = 'marcus_loop.TXT'  # Path to your file
coordinates = load_coordinates(file_path)
directions, total_distance, total_distance = generate_directions_and_distance(coordinates)

# Output the results
for idx, step in enumerate(directions, start=1):
    coord, direction, distance = step  # Unpack each tuple from directions
    print(f"Step {idx}: At {coord}, Direction: {direction}, Distance Traveled: {distance:.2f} km")

print(f"\nTotal Distance Traveled: {total_distance:.2f} kilometers")
