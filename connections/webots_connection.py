from __future__ import annotations

import cv2
import numpy as np
from controller import Robot

from connections import utils
from connections.base import RobotConnection


class WebotsConnection(RobotConnection):
    """Manages the connection to Webots: stepping through time, giving time, and capturing camera frames."""

    def __init__(self, time_step: int = utils.TIME_STEP_MS, camera_name: str = utils.WEBOTS_CAMERA_NAME) -> None:
        self.time_step = time_step
        self.robot = Robot()
        self.camera = self.robot.getDevice(camera_name)
        self.camera.enable(self.time_step)

    def get_time(self) -> float:
        """Returns current simulation time in seconds."""
        return self.robot.getTime()

    def step(self) -> bool:
        """Steps the simulation forward by one time step. Returns False if simulation has ended."""
        return self.robot.step(self.time_step) != -1

    def get_frame(self) -> np.ndarray:
        """Captures a BGR frame from the Webots camera."""
        raw_data = self.camera.getImage()
        image = np.frombuffer(raw_data, np.uint8).reshape(
            (self.camera.getHeight(), self.camera.getWidth(), 4)
        )
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
