from enum import Enum

class Color(Enum):
    EMPTY, BLACK, WHITE = range(3)

class Direction(Enum):
    NW, N, NE, W, E, SW, S, SE = range(8)