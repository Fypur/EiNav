from __future__ import annotations

import cv2
import numpy as np
from controller import Robot

from connections import utils
from connections.base import RobotConnection


class WebotsConnection(RobotConnection):
    """Connexion à Webots : temps de la simulation et images de la caméra."""

    def __init__(self, time_step: int = utils.TIME_STEP_MS, camera_name: str = utils.WEBOTS_CAMERA_NAME) -> None:
        self.time_step = time_step
        self.robot = Robot()
        self.camera = self.robot.getDevice(camera_name)
        self.camera.enable(self.time_step)

    def get_time(self) -> float:
        """Temps de la simulation en secondes."""
        return self.robot.getTime()

    def step(self) -> bool:
        """Avance la simulation d'un pas. False si la simulation est finie."""
        return self.robot.step(self.time_step) != -1

    def get_frame(self) -> np.ndarray:
        """Image BGR de la caméra de Webots."""
        raw_data = self.camera.getImage()
        image = np.frombuffer(raw_data, np.uint8).reshape(
            (self.camera.getHeight(), self.camera.getWidth(), 4)
        )
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
