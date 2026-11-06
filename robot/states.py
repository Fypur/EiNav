"""Modes de mission et états du robot."""
from enum import Enum, auto


class Mode(Enum):
    GRID = auto()
    HUIT = auto()


class RobotState(Enum):
    IDLE = auto()  # moteurs coupés, on attend un ordre
    FOLLOWING = auto()
    SEARCHING_LINE = auto()  # on cherche la ligne perdue
    APPROACHING_INTERSECTION = auto()
    STOP_ON_INTERSECTION = auto()
    TURNING_90 = auto()
    TURNING_MINUS_90 = auto()
    TURNING_180 = auto()
    ALIGNING_LINE = auto()  # fin du demi-tour devant un obstacle : on se recale sur la ligne avec la caméra
    DELIVERING = auto()  # tour complet au point de livraison avant de repartir
    STOPPED = auto()
