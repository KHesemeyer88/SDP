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
    # Convert latitude and longitude from degrees to radians
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)
    
    # Calculate the difference in longitudes
    dlon = lon2 - lon1
    
    # Calculate bearing
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    initial_bearing = math.atan2(x, y)
    
    # Convert bearing from radians to degrees and normalize to 0-360
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360
    
    return compass_bearing

def determine_direction(current_bearing, target_bearing):
    """
    Determine the direction to turn to reach the target bearing from the current bearing.
    
    :param current_bearing: The current heading in degrees
    :param target_bearing: The target heading in degrees
    :return: Instruction on which direction to take
    """
    # Calculate the difference in bearing
    difference = (target_bearing - current_bearing + 360) % 360

    if difference < 10 or difference > 350:
        return "Go straight"
    elif difference > 180:
        return "Turn left"
    elif difference < 180:
        return "Turn right"
    else:
        return "U-turn"

# Example Usage:
# Define two GPS points (latitude, longitude)
point_A = (34.052235, -118.243683)  # Example starting point: Los Angeles
point_B = (36.169941, -115.139832)  # Example destination point: Las Vegas

# Assume an initial heading of 0 degrees (facing north)
current_heading = 0

# Calculate the bearing from point A to point B
target_bearing = calculate_bearing(point_A[0], point_A[1], point_B[0], point_B[1])

# Determine the direction to adjust the current heading towards the target
direction = determine_direction(current_heading, target_bearing)

# Output the result
print(f"Current Heading: {current_heading}°")
print(f"Target Bearing: {target_bearing:.2f}°")
print(f"Direction: {direction}")
