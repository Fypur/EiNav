from enum import Enum, auto


class Direction(Enum):
    EAST = auto()
    WEST = auto()
    NORTH = auto()
    SOUTH = auto()

    @property
    def opposite(self) -> "Direction":
        return {Direction.EAST: Direction.WEST, Direction.WEST: Direction.EAST,
                Direction.NORTH: Direction.SOUTH, Direction.SOUTH: Direction.NORTH}[self]
