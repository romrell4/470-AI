from Enums import Color, Direction

class Square:
    def __init__(self):
        self.piece = EMPTY
        self.neighbor = [None, None, None, None, None, None, None, None]

    def canFlip(self, color, direction):
        #Add logic - recursively check if can flip in given direction
        return False

    def isPlayable(self, color):
        #Add logic - canFlip in at least one direction
        return False

    def playValue(self, color):
        #Add logic - number of flips times 2 plus 1
        return 1

    def play(self, color):
        #Add logic - flip in every possible direction

    def flip(self, color, direction):
        if color == EMPTY:
            raise Exception("Can't remove piece")
        elif self.color == color or self.color == EMPTY:
            return
        elif self.color == BLACK:
            self.color == WHITE
        elif self.color == WHITE:
            self.color == BLACK
        self.neighbor[direction].flip(color)