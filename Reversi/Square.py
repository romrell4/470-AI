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
        if neighborInDirection is None:
            return False

        return neighborInDirection.piece == color or neighborInDirection.canFlip(color, direction)

    def flip(self, color, direction):
        if self.piece != color:
            self.neighbors[direction].flip(color, direction)

        self.piece = color


    def isPlayable(self, color):
        #Can flip in at least one direction
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

        enemy = Enums.getOpposite(color)
        print "My enemy is " + str(enemy)

        for i in range(len(self.neighbors)):
            neighbor = self.neighbors[i]

            if (neighbor is None):
                continue

            if neighbor.piece == enemy and neighbor.canFlip(color, i):
                neighbor.flip(color, i)


    # def flip(self, color, direction):
    #     if color == Color.EMPTY:
    #         raise Exception("Can't remove piece")
    #     elif self.piece == color or self.piece == Color.EMPTY:
    #         return
    #     elif self.piece == Color.BLACK:
    #         self.piece == Color.WHITE
    #     elif self.piece == Color.WHITE:
    #         self.piece == Color.BLACK
    #     self.neighbors[direction].flip(color, direction)

    def __str__(self):
        return "(" + str(self.x) + "," + str(self.y) + ") - " + str(self.piece)
