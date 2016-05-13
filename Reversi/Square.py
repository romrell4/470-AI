import Enums
from Enums import Color, Direction

class Square:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.piece = Color.EMPTY
        self.neighbors = [None, None, None, None, None, None, None, None]

    def canFlip(self, color, direction):
        #Recursively check if can flip in given direction
        neighborInDirection = self.neighbors[direction]
        if self.piece != Enums.getOpposite(color) or neighborInDirection is None:
            return False
        return neighborInDirection.piece == color or neighborInDirection.canFlip(color, direction)

    def flip(self, color, direction):
        oldColor = self.piece
        self.piece = color

        if oldColor != color:
            self.neighbors[direction].flip(color, direction)

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

    def playValue(self, color):
        #Add logic - number of flips times 2 plus 1
        return 1

    def play(self, color):
        #Add logic - flip in every possible direction

        enemy = Enums.getOpposite(color)

        for i in range(len(self.neighbors)):
            neighbor = self.neighbors[i]

            if (neighbor is None):
                continue

            if neighbor.piece == enemy and neighbor.canFlip(color, i):
                self.flip(color, i)

    def __str__(self):
        return "(" + str(self.x) + "," + str(self.y) + ")"
