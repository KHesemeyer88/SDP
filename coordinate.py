import re
import tkinter as tk
from tkinter import filedialog
import math
import random

def load_coordinates(file_path):
    """
    Reads and validates coordinates from the specified text file.
    
    Parameters:
    file_path (str): Path to the text file with coordinates.
    
    Returns:
    list: A list of tuples containing valid (latitude, longitude) coordinates.
    """
    coordinates = []
    with open('C:/Users/Minh/Downloads/test_coord.txt', 'r') as file:
        for line_number, line in enumerate(file, 1):
            # Use regex to match two integers separated by whitespace
            match = re.match(r"^\s*(-?\d+)\s+(-?\d+)\s*$", line)
            if match:
                latitude, longitude = map(int, match.groups())
                
                # Validate latitude and longitude ranges for realistic GPS coordinates
                if -900000000 <= latitude <= 900000000 and -1800000000 <= longitude <= 1800000000:
                    coordinates.append((latitude, longitude))
                else:
                    print(f"Warning: Coordinate out of range on line {line_number}: {latitude}, {longitude}")
            else:
                print(f"Error: Invalid format on line {line_number}: '{line.strip()}'")
    
    return coordinates

# Load and test the coordinates from the file
coordinates = load_coordinates('/mnt/data/test_coord.TXT')

# Output the parsed and validated coordinates
for i, (lat, lon) in enumerate(coordinates):
    print(f"Coordinate {i + 1}: Latitude = {lat}, Longitude = {lon}")

# Check for continuity (if necessary for navigation testing)
def check_continuity(coords):
    """
    Checks if the sequence of coordinates has a reasonable progression.
    For simplicity, checks if the latitude and longitude values change in small increments.
    
    Parameters:
    coords (list): List of tuples containing (latitude, longitude).
    
    Returns:
    bool: True if continuity is maintained, False otherwise.
    """
    for i in range(1, len(coords)):
        lat_diff = abs(coords[i][0] - coords[i - 1][0])
        lon_diff = abs(coords[i][1] - coords[i - 1][1])
        
        # Assuming a small difference threshold for testing continuity
        if lat_diff > 10 or lon_diff > 10:
            print(f"Discontinuity detected between points {i} and {i+1}: Δlat = {lat_diff}, Δlon = {lon_diff}")
            return False
    return True

# Test continuity for navigation
if check_continuity(coordinates):
    print("All coordinates maintain continuity.")
else:
    print("Discontinuity detected in the coordinates.")
