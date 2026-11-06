from __future__ import annotations

import time
from typing import Optional

import numpy as np

from connections import utils
from connections.base import RobotConnection
from vision.camera_pi import RobotCamera


class RealRobotConnection(RobotConnection):
    """Connexion au vrai robot : temps et images de la caméra."""

    def __init__(self) -> None:
        self.camera = RobotCamera()

    def get_time(self) -> float:
        """Temps actuel en secondes."""
        return time.time()

    def step(self) -> bool:
        """Attend un pas de temps."""
        time.sleep(utils.TIME_STEP_MS / 1000.0)
        return True

    def get_frame(self) -> Optional[np.ndarray]:
        """Image BGR de la caméra du Pi."""
        return self.camera.read()

    def publish_frame(self, frame: np.ndarray) -> None:
        """Envoie l'image traitée dans le flux vidéo."""
        self.camera.publish(frame)

    def close(self) -> None:
        """Ferme la caméra."""
        self.camera.close()
