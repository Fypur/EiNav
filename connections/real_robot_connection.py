from __future__ import annotations

import time
from typing import Optional

import numpy as np

from connections.base import RobotConnection
from vision.camera_pi import RobotCamera


class RealRobotConnection(RobotConnection):
    """Manages the connection on the physical robot: stepping time, giving time, and capturing camera frames."""

    def __init__(self, time_step: int = 64) -> None:
        self.time_step = time_step
        self.camera = RobotCamera()

    def get_time(self) -> float:
        """Returns current epoch time in seconds."""
        return time.time()

    def step(self) -> bool:
        """Paces execution for one time step."""
        time.sleep(self.time_step / 1000.0)
        return True

    def get_frame(self) -> Optional[np.ndarray]:
        """Captures a BGR frame from the Raspberry Pi camera."""
        return self.camera.read()

    def publish_frame(self, frame: np.ndarray) -> None:
        """Sends the processed image to the video stream."""
        self.camera.publish(frame)

    def close(self) -> None:
        """Closes the camera."""
        self.camera.close()
