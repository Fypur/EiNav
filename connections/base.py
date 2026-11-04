"""Interface commune au robot réel et à la simulation Webots."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

import numpy as np


class RobotConnection(ABC):
    @abstractmethod
    def get_time(self) -> float:
        """Temps courant en secondes."""

    @abstractmethod
    def step(self) -> bool:
        """Avance d'un pas de temps. Renvoie False quand il faut s'arrêter."""

    @abstractmethod
    def get_frame(self) -> Optional[np.ndarray]:
        """Image BGR de la caméra, ou None si la lecture échoue."""

    def publish_frame(self, frame: np.ndarray) -> None:
        """Diffuse l'image traitée au tableau de bord (sans effet par défaut)."""

    def close(self) -> None:
        """Libère les ressources (sans effet par défaut)."""
