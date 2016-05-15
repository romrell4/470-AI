from Board import Board
from Square import Square
import Enums
from Enums import Color
from Tree import Tree

class Reversi:

    def start(self):
        
        self.config = self.getConfig()
        
        while(True):
            self.board = Board(self.getBoardSize(), self.config)
            self.depth = self.getSearchDepth()
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
                    print Color.str[self.turn] + " has no legal moves"
                    
                    #Check if the other player also couldn't play. If so, the game is over
                    if done:
                        score = self.board.getScore(False)
                        if score[Color.BLACK] > score[Color.WHITE]:
                            print Color.str[Color.BLACK] + " wins!"
                        elif score[Color.BLACK] < score[Color.WHITE]:
                            print Color.str[Color.WHITE] + " wins!"
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

    def getConfig(self):
        print ""
        for i in range(len(Color.chr)):
            conf_str = str(i + 1) + ": "
            for j in range(len(Color.chr[i])):
                conf_str += "(" + Color.chr[i][j] + " = " + Color.str[j] + ") "
            print conf_str
        while(True):
            config = raw_input('Select a board configuration: ')
            if not config.isdigit(): continue
            config = int(config) - 1
            if config in range(len(Color.chr)): return config
    
    def getBoardSize(self):
        print ""
        print "4: minimum board width"
        print "8: standard board width"
        print "63: maximum board width"
        while(True):
            size = raw_input('Select a board width: ')
            if not size.isdigit(): continue
            size = int(size)
            if size >= 4 and size <= 63: return size
            
    def getSearchDepth(self):
        print ""
        while(True):
            depth = raw_input('Select how many moves ahead the computer will look: ')
            if not depth.isdigit(): continue
            depth = int(depth)
            if depth >= 0: return depth
    
    def getColorSelection(self):
        print ""
        print str(Color.BLACK) + ":", Color.str[Color.BLACK]
        print str(Color.WHITE) + ":", Color.str[Color.WHITE]
        while(True):
            color = raw_input('Select a color (' + Color.str[Color.BLACK] + ' goes first): ')
            if not color.isdigit(): continue
            color = int(color)
            if color == Color.BLACK or color == Color.WHITE: return color
                
    def playAgain(self):
        again = raw_input('Play again? (Y/N): ')
        again = again.upper()[0]
        return again == 'Y'

    def getChoiceUser(self, possibilities):
        count = 1
        for pos in possibilities:
            print str(count) + ":", \
            "(" + Enums.getAlpha(pos.x) + "," + str(pos.y+1) + ")"
            count = count + 1
        while(True):
            move = raw_input('Enter your move: ')
            if not move.isdigit(): continue
            choice = int(move)
            if choice != 0 and choice < len(possibilities)+1:
                return possibilities[choice-1]

    def getChoiceComputer(self, possibilities):
        print Color.str[self.ai] + " is thinking..."
        choice = Tree(self.board, self.ai).getBest(self.depth)
        return possibilities[choice]

    def endTurn(self):
        print self.board
        # raw_input("Continue?")
        self.turn = Enums.getOpposite(self.turn)

game = Reversi()
game.start()
