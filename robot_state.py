from enum import Enum, auto


class RobotState(Enum):
    FOLLOWING = auto()
    APPROACHING_INTERSECTION = auto()
    STOP_ON_INTERSECTION = auto()
    TURNING_90 = auto()
    TURNING_MINUS_90 = auto()
    TURNING_180 = auto()
    STOPPED = auto()