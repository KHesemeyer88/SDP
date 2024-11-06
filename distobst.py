import math
import random

class ObstacleDetector:
    def __init__(self, obstacle_distance=None):
        """
        Initialize the detector. Optionally set an initial obstacle distance.
        :param obstacle_distance: Initial distance to obstacle in meters
        """
        self.obstacle_distance = obstacle_distance if obstacle_distance is not None else random.uniform(1, 50)

    def get_distance_to_obstacle(self):
        """
        Simulate getting the current distance to the nearest obstacle.
        For a real sensor, this would be replaced with a reading function.
        :return: Distance to the obstacle in meters
        """
        # Simulate distance reading, normally this would come from a sensor
        self.obstacle_distance = random.uniform(0.5, 20)  # Simulated reading in meters
        return self.obstacle_distance

    def report_distance_to_obstacle(self):
        """
        Report the current distance to an obstacle and give feedback if too close.
        """
        distance = self.get_distance_to_obstacle()
        
        # Set safety threshold
        safety_threshold = 5.0

        if distance <= safety_threshold:
            print(f"Warning: Obstacle very close! Distance to obstacle: {distance:.2f} meters")
        else:
            print(f"Distance to obstacle: {distance:.2f} meters - Safe distance maintained")

# Example usage
detector = ObstacleDetector()

# Simulate periodic obstacle distance reporting
for _ in range(10):  # Run for 10 readings
    detector.report_distance_to_obstacle()