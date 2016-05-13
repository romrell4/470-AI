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


        # This is greedy method
        # diffs = []

        # for option in self.options:
        #     # print "Option: " + str(option)
        #     diffs.append(self.getResult(option).root.getDiff(self.color))

        # maxIndex = 0
        # for i in range(len(diffs)):
        #     if diffs[i] > diffs[maxIndex]:
        #         maxIndex = i

        # return maxIndex




        #This is for iterative pruning method
        # bests = []

        # for option in self.options:
        #     theirTree = self.getResult(option)

        #     maxIndex = 0
        #     theirDiffs = []
        #     theirOptions = theirTree.options

        #     print theirTree.root
        #     print "The value for this tree is " + str(theirTree.root.getDiff(self.color))

        #     for theirOption in theirOptions:
        #         print theirOption
        #         ourTree = theirTree.getResult(theirOption)

        #         ourOptions = ourTree.options

        #         print ourTree.root
        #         print "The value for this tree is " + str(ourTree.root.getDiff(self.color))

        #         # theirDiffs.append(ourTree.root.getDiff(theirTree.color))

        #         for ourOption in ourOptions:
        #             print ourOption

        #             theirTree2 = ourTree.getResult(ourOption)

        #             print theirTree2.root
        #             print "The value for this tree is " + str(theirTree2.root.getDiff(self.color))



        #     print ""

        best = self.checkChildren(2, self.color)

        print best

        exit()

    def checkChildren(self, depth, color):
        if depth == 0:
            print "I am the bottom! My value is: " + str(self.root.getDiff(color))
            return self.root.getDiff(color)

        bests = []

        for option in self.options:
            tree = self.getResult(option)
            bests.append(tree.checkChildren(depth - 1, color))

        print "Here is the best of all my children: " + str(max(bests))
        return bests

    def getResult(self, option):
        return Tree(self.root.getConfig(option.x, option.y, self.color), self.enemy)

    def getState(self):
        return self.root.getDiff(self.color)