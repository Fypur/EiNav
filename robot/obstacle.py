"""Freinage devant un obstacle avec le capteur ultrason.

Entre OBSTACLE_SLOWDOWN_CM et OBSTACLE_STOP_CM la vitesse est proportionnelle à la distance
restante. On ne descend pas sous FORWARD_SPEED, sinon les moteurs peuvent caler avant l'arrêt.
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
    NO_MEASURE = auto()  # capteur muet, on l'ignore et on continue la mission


class ObstacleStatus(NamedTuple):
    state: ObstacleState
    distance_cm: Optional[int]  # None si pas de mesure valide


class ObstacleGuard:
    """Calcule la vitesse max autorisée à partir des mesures de distance.

    On prend le min des dernières mesures. Un écho raté donne « rien devant » et ne doit pas
    relancer le robot, alors qu'un obstacle proche doit le freiner dès la première mesure.

    Une fois arrêté, on ne repart qu'au-dessus d'OBSTACLE_RESUME_CM, sinon une mesure qui hésite
    entre 20 et 21 cm fait avancer le robot par à-coups.

    Si aucune des dernières mesures n'est valide on ignore le capteur et on continue la mission.
    Une seule mesure ratée ne suffit pas, ça peut juste être un raté de la liaison.
    """

    def __init__(self) -> None:
        self._readings: Deque[Optional[int]] = deque(maxlen=utils.OBSTACLE_FILTER_READINGS)
        self._blocked = False

    @property
    def _nearest(self) -> Optional[int]:
        return min((reading for reading in self._readings if reading is not None), default=None)

    def update(self, distance_cm: Optional[int]) -> float:
        """Ajoute une mesure (None ou 0 si pas de mesure) et renvoie la vitesse max."""
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
