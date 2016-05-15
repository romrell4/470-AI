from Board import Board
from Square import Square
import Enums
from Enums import Color

UNKNOWN = -1
INDEX, SCORE = range(2)

class Tree:
    def __init__(self, board, color):
        self.board = board
        self.color = color
        self.enemy = Enums.getOpposite(color)
        self.options = board.getPlayableSquares(color)
    
    def branch(self, option):
        if option is None: return Tree(self.board, self.enemy)
        return Tree(self.board.getConfig(option.x, option.y, self.color), self.enemy)

    def getBest(self, depth):
        max = [UNKNOWN, [0, 0, 0]]
        max[SCORE][self.color] = self.board.weight
        best = self.checkBranches(depth, self.color, max)
        return best[INDEX]

    def checkBranches(self, depth, color, root):
        if depth == 0:
            return [UNKNOWN, self.board.getScore(True)]
        
        enemy = Enums.getOpposite(color)
        
        if color == self.color: # Maximize minimum
            # print "Maximizing Minimum (basically, our turn)"
            min = [UNKNOWN, [0, 0, 0]]
            min[SCORE][enemy] = self.board.weight

            if len(self.options) == 0:
                return self.branch(None).checkBranches(depth - 1, color, min)

            for index in range(len(self.options)):
                option = self.options[index]
                tree = self.branch(option)
                score = tree.checkBranches(depth - 1, color, min)[SCORE]
                if score[enemy] == 0:
                    return [index, score]

                #Check the diff to see if we should update our maximized score
                if score[color] - score[enemy] > min[SCORE][color] - min[SCORE][enemy]:
                    min = [index, score]

                #Check to see if you're already doing better than the root would be without you
                if min[SCORE][color] - min[SCORE][enemy] > root[SCORE][color] - root[SCORE][enemy]:
                    #if index != len(self.options) - 1:
                    #    print "I am pruning at depth " + str(depth) + "!"
                    return min

            return min
        
        else: # Minimize maximum
            # print "Minimizing maximum (basically, not our turn)"
            max = [UNKNOWN, [0, 0, 0]]
            max[SCORE][color] = self.board.weight

            if len(self.options) == 0:
                return self.branch(None).checkBranches(depth - 1, color, max)

            for index in range(len(self.options)):
                option = self.options[index]
                tree = self.branch(option)
                score = tree.checkBranches(depth - 1, color, max)[SCORE]
                if score[color] == 0:
                    return [index, score]

                #Check the diff to see if we should update our minimized score
                if score[color] - score[enemy] < max[SCORE][color] - max[SCORE][enemy]:
                    max = [index, score]

                #Check to see if you're already doing worse than the root would be without you
                if max[SCORE][color] - max[SCORE][enemy] < root[SCORE][color] - root[SCORE][enemy]:
                    #if index != len(self.options) - 1:
                    #    print "I am pruning at depth " + str(depth) + "!"
                    return max

            return max

    '''
    def getBestGreedy(self):
        # This is greedy method
        diffs = []

        for option in self.options:
            # print "Option: " + str(option)
            diffs.append(self.branch(option).board.getDiff(self.color))

        maxIndex = 0
        for i in range(len(diffs)):
            if diffs[i] > diffs[maxIndex]:
                maxIndex = i

        return maxIndex

    def getBestIterative(self):
        #This is for iterative pruning method
        bests = []

        for option in self.options:
            theirTree = self.branch(option)

            maxIndex = 0
            theirDiffs = []
            theirOptions = theirTree.options

            print theirTree.board
            print "The value for this tree is " + str(theirTree.board.getDiff(self.color))

            for theirOption in theirOptions:
                print theirOption
                ourTree = theirTree.branch(theirOption)

                ourOptions = ourTree.options

                print ourTree.board
                print "The value for this tree is " + str(ourTree.board.getDiff(self.color))

                # theirDiffs.append(ourTree.board.getDiff(theirTree.color))

                for ourOption in ourOptions:
                    print ourOption

                    theirTree2 = ourTree.branch(ourOption)

                    print theirTree2.board
                    print "The value for this tree is " + str(theirTree2.board.getDiff(self.color))



            print ""
   '''
