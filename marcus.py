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
    Load coordinates from a text file.
    
    :param file_path: Path to the text file containing coordinates
    :return: List of (latitude, longitude) tuples
    """
    coordinates = []
    with open('C:/Users/Minh/Downloads/marcus_loop.txt', 'r') as file:
        for line in file:
            lat, lon = map(float, line.strip().split())
            coordinates.append((lat, lon))
    return coordinates

def generate_directions(coordinates):
    """
    Generate directional instructions based on a sequence of GPS coordinates.
    
    :param coordinates: List of (latitude, longitude) tuples
    :return: List of directions after each measurement
    """
    directions = []
    if len(coordinates) < 2:
        return directions

    # Initialize the starting bearing
    current_bearing = 0  # Assume initial heading north
    
    for i in range(1, len(coordinates)):
        # Calculate the target bearing from the current point to the next
        target_bearing = calculate_bearing(
            coordinates[i-1][0], coordinates[i-1][1],
            coordinates[i][0], coordinates[i][1]
        )
        
        # Determine direction based on current and target bearing
        direction = determine_direction(current_bearing, target_bearing)
        
        # Append the direction to the list and update current bearing
        directions.append((coordinates[i], direction))
        current_bearing = target_bearing
    
    return directions

# Load coordinates from file and generate directions
coordinates = load_coordinates('/mnt/data/marcus_loop.TXT')
directions = generate_directions(coordinates)

# Output the results
for idx, (coord, direction) in enumerate(directions, start=1):
    print(f"Step {idx}: At {coord}, Direction: {direction}")
