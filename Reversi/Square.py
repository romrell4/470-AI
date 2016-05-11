import Enums
from Enums import Color, Direction

class Square:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.piece = Color.EMPTY
        self.neighbors = [None, None, None, None, None, None, None, None]

    def canFlip(self, color, direction):
        #Add logic - recursively check if can flip in given direction
        neighborInDirection = self.neighbors[direction]
        if (neighborInDirection is None):
            return False

        return neighborInDirection.piece == color or neighborInDirection.canFlip(color, direction)

    def flip(self, color, direction):
        )
        self.piece


    def isPlayable(self, color):
        #Add logic - canFlip in at least one direction
        if self.piece != Color.EMPTY:
            # print "I am taken!"
            return False

        enemy = Enums.getOpposite(color)
        # print "My enemy is: " + str(enemy)

        for i in range(len(self.neighbors)):
            neighbor = self.neighbors[i]

            if neighbor is None:
                continue

            if neighbor.piece == enemy and neighbor.canFlip(color, i):
                # print "Found an enemy neighbor at " + str(i)
                return True

        return False

    def playValue(self, color):
        #Add logic - number of flips times 2 plus 1
        return 1

    def play(self, color):
        #Add logic - flip in every possible direction
        for neighbor in self.neighbors:
            if (neighbor is None):
                continue
            neighbor.flip()


    def flip(self, color, direction):
        if color == Color.EMPTY:
            raise Exception("Can't remove piece")
        elif self.color == color or self.color == Color.EMPTY:
            return
        elif self.color == Color.BLACK:
            self.color == Color.WHITE
        elif self.color == Color.WHITE:
            self.color == Color.BLACK
        self.neighbor[direction].flip(color)

    def __str__(self):
        return "(" + str(self.x) + "," + str(self.y) + ") - " + str(self.piece)
