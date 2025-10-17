import math
import threading

class MockMyCobot:
    """
    A mock class that simulates the behavior of the pymycobot.MyCobot object
    for testing purposes without actual hardware.
    """
    def __init__(self, port, baudrate):
        # The constructor takes port and baudrate to match the real class signature,
        # but doesn't use them.
        self._angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # Use a lock to simulate thread-safe access to joint angles
        self._lock = threading.Lock()
        print(f"Initialized MockMyCobot on fake port '{port}' with baudrate {baudrate}")

    def is_controller_connected(self):
        """Simulates a successful connection."""
        return 1  # The real class returns 1 for connected

    def get_angles(self):
        """Returns the current simulated joint angles in degrees."""
        with self._lock:
            return list(self._angles) # Return a copy

    def send_angles(self, angles, speed):
        """Sets the new simulated joint angles in degrees."""
        with self._lock:
            print(f"MockMyCobot: Moving to angles {angles} with speed {speed}")
            self._angles = list(angles)

    def degrees_to_radians(self, degree):
        """Converts a single degree value to radians."""
        return math.radians(degree)

    def radians_to_degrees(self, radian):
        """Converts a single radian value to degrees."""
        return math.degrees(radian)
