import time
import numpy as np
from camera_pi import RobotCamera


class RealRobotConnection:
    """Manages the connection on the physical robot: stepping time, giving time, and capturing camera frames."""

    def __init__(self, time_step: int = 64):
        self.time_step = time_step
        self.camera = RobotCamera()

    def get_time(self) -> float:
        """Returns current epoch time in seconds."""
        return time.time()

    def step(self) -> bool:
        """Paces execution for one time step."""
        time.sleep(self.time_step / 1000.0)
        return True

    def get_frame(self) -> np.ndarray:
        """Captures a BGR frame from the Raspberry Pi camera."""
        return self.camera.read()

    def close(self):
        """Closes the camera."""
        self.camera.close()
