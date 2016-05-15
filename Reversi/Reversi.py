from Board import Board
from Square import Square
import Enums
from Enums import Color
from Tree import Tree

class Reversi:
    '''
    def __init__(self):
        self.board = Board()
        self.player = Color.BLACK
        self.ai = Enums.getOpposite(self.player)
        self.turn = Color.BLACK
    '''

    def start(self):
        
        while(True):
            self.board = Board()
            self.player = self.getColorSelection()
            self.ai = Enums.getOpposite(self.player)
            self.turn = Color.BLACK
        
            # Print the board to start
            print self.board

            done = False

            while(True):
                #Check for all posssible locations
                possibilities = self.board.getPlayableSquares(self.turn)

                #Check if you had no possible locations
                if len(possibilities) == 0:
                    print Color.str[self.turn]+" has no legal moves"
                    
                    #Check if the other player also couldn't play. If so, the game is over
                    if done:
                        score = self.board.getScore(False)
                        if score[Color.BLACK] > score[Color.WHITE]:
                            print Color.str[Color.BLACK]+" wins!"
                        elif score[Color.BLACK] < score[Color.WHITE]:
                            print Color.str[Color.WHITE]+" wins!"
                        else:
                            print "Tie game!"
                        break

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

            if not self.playAgain(): return

    def getColorSelection(self):
        print str(Color.BLACK)+":", Color.str[Color.BLACK]
        print str(Color.WHITE)+":", Color.str[Color.WHITE]
        
        while(True):
            color = raw_input('Select a color: ')
            color = int(color)
            if color == Color.BLACK or color == Color.WHITE:
                break
        return color
                
    def playAgain(self):
        again = raw_input('Play again? (Y/N): ')
        again = again.upper()[0]
        return again == 'Y'

    def getChoiceUser(self, possibilities):
        count = 1
        for pos in possibilities:
            print str(count)+":", "("+Enums.getAlpha(pos.x)+","+str(pos.y+1)+")"
            count = count + 1

        while(True):
            move = raw_input('Enter your move: ')
            choice = int(move)
            if choice != 0 and choice < len(possibilities)+1:
                break
                    
        return possibilities[choice-1]

    def getChoiceComputer(self, possibilities):
        print Color.str[self.ai]+" is thinking..."
        choice = Tree(self.board, self.ai).getBest()
        return possibilities[choice]

    def endTurn(self):
        print self.board
        # raw_input("Continue?")
        self.turn = Enums.getOpposite(self.turn)

game = Reversi()
game.start()
