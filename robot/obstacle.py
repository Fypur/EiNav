"""Freinage devant un obstacle, à partir des mesures du capteur ultrason.

Entre OBSTACLE_SLOWDOWN_CM et OBSTACLE_STOP_CM, la vitesse est proportionnelle à la distance
qui reste à parcourir, sans descendre sous FORWARD_SPEED : plus lentement, les moteurs risqueraient
de caler avant d'atteindre la distance d'arrêt.
"""
from __future__ import annotations

import math
from collections import deque
from enum import Enum, auto
from typing import Deque, NamedTuple, Optional

from robot import utils


class ObstacleState(Enum):
    CLEAR = auto()
    SLOWING = auto()
    BLOCKED = auto()
    NO_MEASURE = auto()  # capteur muet : ignoré, le robot poursuit sa mission


class ObstacleStatus(NamedTuple):
    state: ObstacleState
    distance_cm: Optional[int]  # None : aucune mesure valide


class ObstacleGuard:
    """Transforme les mesures de distance en vitesse maximale autorisée.

    On garde le minimum des dernières mesures : un écho manqué se lit « rien devant » et ne doit
    pas relancer le robot, alors qu'un obstacle proche doit le freiner dès la première mesure.
    Une fois arrêté, le robot ne repart qu'au-delà d'OBSTACLE_RESUME_CM : sinon, une mesure
    oscillant entre 20 et 21 cm le ferait avancer par à-coups vers l'obstacle.

    Sans aucune mesure parmi les dernières, le capteur est ignoré et le robot poursuit sa mission.
    Une absence isolée ne suffit pas : un raté de la liaison ne doit pas effacer un obstacle proche.
    """

    def __init__(self) -> None:
        self._readings: Deque[Optional[int]] = deque(maxlen=utils.OBSTACLE_FILTER_READINGS)
        self._blocked = False

    @property
    def _nearest(self) -> Optional[int]:
        return min((reading for reading in self._readings if reading is not None), default=None)

    def update(self, distance_cm: Optional[int]) -> float:
        """Prend en compte une mesure (None ou 0 : pas de mesure) et renvoie la vitesse maximale."""
        self._readings.append(distance_cm or None)
        nearest = self._nearest
        if nearest is None:
            self._blocked = False
            return math.inf

        if nearest <= utils.OBSTACLE_STOP_CM:
            self._blocked = True
        elif nearest >= utils.OBSTACLE_RESUME_CM:
            self._blocked = False

        if self._blocked:
            return 0.0
        if nearest >= utils.OBSTACLE_SLOWDOWN_CM:
            return math.inf
        remaining = (nearest - utils.OBSTACLE_STOP_CM) / (utils.OBSTACLE_SLOWDOWN_CM - utils.OBSTACLE_STOP_CM)
        return max(utils.FORWARD_SPEED, utils.BASE_SPEED * remaining)

    @property
    def status(self) -> ObstacleStatus:
        nearest = self._nearest
        if nearest is None:
            return ObstacleStatus(ObstacleState.NO_MEASURE, None)
        if self._blocked:
            return ObstacleStatus(ObstacleState.BLOCKED, nearest)
        if nearest < utils.OBSTACLE_SLOWDOWN_CM:
            return ObstacleStatus(ObstacleState.SLOWING, nearest)
        return ObstacleStatus(ObstacleState.CLEAR, nearest)
