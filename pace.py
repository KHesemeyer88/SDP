def calculate_pace(distance_km, time_minutes):
    """
    Calculate running pace based on distance and time.
    
    :param distance_km: Distance covered in kilometers (km)
    :param time_minutes: Total time in minutes
    :return: Pace in minutes per kilometer (min/km)
    """
    if distance_km <= 0:
        raise ValueError("Distance must be greater than zero.")
    if time_minutes <= 0:
        raise ValueError("Time must be greater than zero.")
        
    pace = time_minutes / distance_km
    return pace  # Pace in minutes per kilometer


def pace_difference(current_pace, target_pace):
    """
    Calculate the difference between current and target pace.
    
    :param current_pace: Current pace in minutes per kilometer
    :param target_pace: Target pace in minutes per kilometer
    :return: Difference in pace, positive if slower than target, negative if faster
    """
    return current_pace - target_pace


def time_to_target(distance_km, current_time, target_pace):
    """
    Calculate the time a runner would need to meet a target pace.
    :param distance_km: Distance covered in kilometers
    :param current_time: Current running time in minutes
    :param target_pace: Desired pace in minutes per kilometer
    :return: Suggested time to finish the current distance to meet target pace
    """
    target_time = distance_km * target_pace
    return target_time - current_time  # Positive if slower than target, negative if ahead

# Example usage
distance_km = 5.0  # Distance in kilometers
time_minutes = 27  # Total running time in minutes
target_pace = 5.0  # Target pace in minutes per kilometer

current_pace = calculate_pace(distance_km, time_minutes)
pace_diff = pace_difference(current_pace, target_pace)
time_adjustment = time_to_target(distance_km, time_minutes, target_pace)

print(f"Current pace: {current_pace:.2f} min/km")
print(f"Pace difference: {pace_diff:.2f} min/km")
print(f"Time adjustment needed to meet target pace: {time_adjustment:.2f} minutes")

