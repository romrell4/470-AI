from enum import Enum

class Color(Enum):
    EMPTY = 0
    BLACK = 1
    WHITE = 2

class Direction(Enum):
    NW = 0
    N = 1
    NE = 2
    W = 3
    E = 4
    SW = 5
    S = 6
    SE = 7