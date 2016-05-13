from Board import Board
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
        best = options[0]
        return [best.x, best.y]

    def getResult(self, option):
        return Tree(self.root.getConfig(option.x, option.y, self.color) self.enemy)

    def getState(self):
        return self.root.getDiff(self.color)