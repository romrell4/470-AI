import Enums
from Enums import Color, Direction

class Square:
    def __init__(self, x, y, value):
        self.x = x
        self.y = y
        self.value = value
        self.piece = Color.EMPTY
        self.neighbors = [None, None, None, None, None, None, None, None]

    def canFlip(self, color, direction):
        #Recursively check if can flip in given direction
        neighborInDirection = self.neighbors[direction]
        if self.piece != Color.opp[color] or neighborInDirection is None:
            return False
        return neighborInDirection.piece == color or neighborInDirection.canFlip(color, direction)

    def flip(self, color, direction):
        if self.piece == color:
            return
        self.neighbors[direction].flip(color, direction)
        self.piece = color
    

    def isPlayable(self, color):
        #Can flip in at least one direction
        if self.piece != Color.EMPTY:
            return False

        for i in range(len(self.neighbors)):
            neighbor = self.neighbors[i]

            if neighbor is None:
                continue

            if neighbor.canFlip(color, i):
                return True

        return False

    def play(self, color):
        #flip in every possible direction
        if self.piece != Color.EMPTY:
            return
        self.piece = color

        for i in range(len(self.neighbors)):
            neighbor = self.neighbors[i]

            if neighbor is None:
                continue

            if neighbor.canFlip(color, i):
                neighbor.flip(color, i)

    def __str__(self):
        return "(" + str(self.x) + "," + str(self.y) + ")"
