from Board import Board
from Square import Square
import Enums
from Enums import Color

class Tree:
    def __init__(self, board, color):
        self.root = board
        self.color = color
        self.enemy = Enums.getOpposite(color)
        self.options = board.getPlayableSquares(color)

    def getBest(self):
        #TODO: choose a best option
        diffs = []

        for option in self.options:
            # print "Option: " + str(option)
            diffs.append(self.getResult(option).root.getDiff(self.color))

        maxIndex = 0
        for i in range(len(diffs)):
            if diffs[i] > diffs[maxIndex]:
                maxIndex = i

        return maxIndex

    def getResult(self, option):
        return Tree(self.root.getConfig(option.x, option.y, self.color), self.enemy)

    def getState(self):
        return self.root.getDiff(self.color)