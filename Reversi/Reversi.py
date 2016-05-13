from Board import Board
from Square import Square
import Enums
from Enums import Color
from Tree import Tree

class Reversi:
    def __init__(self):
        self.board = Board()
        self.player = Color.WHITE
        self.ai = Enums.getOpposite(self.player)
        self.turn = self.player

    def start(self):
        # Print the board to start
        print self.board

        done = False

        while(True):
            #Check for all posssible locations
            possibilities = self.board.getPlayableSquares(self.turn)

            #Check if you had no possible locations
            if len(possibilities) == 0:
                #Check if the other player also couldn't play. If so, the game is over
                if done:
                    return

                #Set the variable to notify the program that you couldn't go
                done = True
                self.endTurn()
                continue

            #Set the variable to notify the program that you could go
            done = False
            # compoute best move, by alpha beta
            #Get the best choice and play it

            if self.turn == self.ai:
                choice = self.getChoiceComputer(possibilities)
            else:
                choice = self.getChoiceUser(possibilities)

            self.board.play(choice.x, choice.y, self.turn)
            self.endTurn()

    def getChoiceUser(self, possibilities):
        count = 1
        for pos in possibilities:
            print str(count)+":", "("+self.getAlpha(pos.x)+","+str(pos.y+1)+")"
            count = count + 1

        while(True):
            move = raw_input('Enter your move: ')
            choice = int(move)
            if choice != 0 and choice < len(possibilities)+1:
                break
                    
        return possibilities[choice-1]

    def getChoiceComputer(self, possibilities):
        #TODO: Add logic to choose the best possibility
        choice = Tree(self.board, self.ai).getBest()
        return possibilities[choice]

    def endTurn(self):
        print self.board
        # raw_input("Continue?")
        self.turn = Enums.getOpposite(self.turn)
    
    def getAlpha(self, pos):
        return chr(pos + ord('A'))

game = Reversi()
game.start()
