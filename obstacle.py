import RPi.GPIO as GPIO
import time

class CornerObstacleDetector:
    def __init__(self, trigger_pin_front, echo_pin_front, trigger_pin_side, echo_pin_side, threshold_distance=30):
        """
        Initialize the obstacle detector with front and side sonar sensors.

        :param trigger_pin_front: GPIO pin for the front sensor's trigger
        :param echo_pin_front: GPIO pin for the front sensor's echo
        :param trigger_pin_side: GPIO pin for the side sensor's trigger
        :param echo_pin_side: GPIO pin for the side sensor's echo
        :param threshold_distance: Distance threshold in centimeters for obstacle detection
        """
        self.trigger_pin_front = trigger_pin_front
        self.echo_pin_front = echo_pin_front
        self.trigger_pin_side = trigger_pin_side
        self.echo_pin_side = echo_pin_side
        self.threshold_distance = threshold_distance

        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin_front, GPIO.OUT)
        GPIO.setup(self.echo_pin_front, GPIO.IN)
        GPIO.setup(self.trigger_pin_side, GPIO.OUT)
        GPIO.setup(self.echo_pin_side, GPIO.IN)
        
        # Initialize trigger pins
        GPIO.output(self.trigger_pin_front, GPIO.LOW)
        GPIO.output(self.trigger_pin_side, GPIO.LOW)
        time.sleep(2)

    def measure_distance(self, trigger_pin, echo_pin):
        """
        Measure distance using the sonar sensor specified by trigger and echo pins.
        """
        # Send pulse
        GPIO.output(trigger_pin, True)
        time.sleep(0.00001)
        GPIO.output(trigger_pin, False)

        # Measure the pulse duration
        while GPIO.input(echo_pin) == 0:
            pulse_start = time.time()
        while GPIO.input(echo_pin) == 1:
            pulse_end = time.time()

        # Calculate the distance
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # distance in cm
        return round(distance, 2)

    def detect_corner_obstacle(self):
        """
        Detect if there's an obstacle at a corner using front and side sonar sensors.
        """
        # Measure distances from both sensors
        distance_front = self.measure_distance(self.trigger_pin_front, self.echo_pin_front)
        distance_side = self.measure_distance(self.trigger_pin_side, self.echo_pin_side)

        # Check if obstacle is detected only by side sensor within threshold
        if distance_front > self.threshold_distance and distance_side < self.threshold_distance:
            print(f"Corner detected! Obstacle at {distance_side} cm on the side but clear in front.")
        elif distance_front < self.threshold_distance and distance_side < self.threshold_distance:
            print(f"Obstacle detected directly in front at {distance_front} cm and at side at {distance_side} cm.")
        else:
            print(f"Safe: Front distance {distance_front} cm, Side distance {distance_side} cm.")

    def cleanup(self):
        """
        Clean up GPIO settings after detecting.
        """
        GPIO.cleanup()

# Example usage
try:
    detector = CornerObstacleDetector(
        trigger_pin_front=18,
        echo_pin_front=24,
        trigger_pin_side=23,
        echo_pin_side=25,
        threshold_distance=30
    )

    for _ in range(10):  # Run for 10 readings
        detector.detect_corner_obstacle()
        time.sleep(1)

finally:
    detector.cleanup()
