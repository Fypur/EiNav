"""Classe de base commune au vrai robot et à Webots."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

import numpy as np


class RobotConnection(ABC):
    @abstractmethod
    def get_time(self) -> float:
        """Temps actuel en secondes."""

    @abstractmethod
    def step(self) -> bool:
        """Avance d'un pas de temps. False quand il faut s'arrêter."""

    @abstractmethod
    def get_frame(self) -> Optional[np.ndarray]:
        """Image BGR de la caméra, None si la lecture rate."""

    def publish_frame(self, frame: np.ndarray) -> None:
        """Envoie l'image traitée à l'interface (ne fait rien par défaut)."""

    def close(self) -> None:
        """Libère les ressources (ne fait rien par défaut)."""
