"""Modes de mission et états de la machine à états du robot."""
from enum import Enum, auto


class Mode(Enum):
    GRID = auto()
    HUIT = auto()


class RobotState(Enum):
    IDLE = auto()  # moteurs coupés, en attente d'un ordre de l'interface
    FOLLOWING = auto()
    SEARCHING_LINE = auto()  # recherche de la ligne perdue
    APPROACHING_INTERSECTION = auto()
    STOP_ON_INTERSECTION = auto()
    TURNING_90 = auto()
    TURNING_MINUS_90 = auto()
    TURNING_180 = auto()
    DELIVERING = auto()  # tour complet sur le point de livraison, avant le retour
    STOPPED = auto()
