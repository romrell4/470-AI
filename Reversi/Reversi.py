from Board import Board
from Square import Square
import Enums
from Enums import Color
from Tree import Tree

DEFAULT_BOARD_SIZE = 8
DEFAULT_DISPLAY = 0
DEFAULT_DEPTH = 5

class Reversi:

    def start(self):
        
        self.boardSize = DEFAULT_BOARD_SIZE
        self.display = DEFAULT_DISPLAY
        self.depth = DEFAULT_DEPTH
        
        while(True):
            option = self.getOption()
            if option == 0:
                self.playGame()
            elif option == 1:
                while(True):
                    setting = self.getSetting()
                    if setting == 0:
                        self.display = self.getDisplay()
                    elif setting == 1:
                        self.boardSize = self.getBoardSize()
                    elif setting == 2:
                        self.depth = self.getSearchDepth()
                    else:
                        break
            else:
                return

    def playGame(self):
        
        while(True):
            self.player = self.getColorSelection()
            self.ai = Color.opp[self.player]
            self.turn = Color.BLACK
            self.board = Board(self.boardSize, self.display)
            
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
                    if choice is None: return
                
                self.board.play(choice.x, choice.y, self.turn)
                self.endTurn()
            
            if not self.playAgain(): return

    def getOption(self):
        print ""
        print "1: New game"
        print "2: Settings"
        print "3: Quit"
        while(True):
            opt = raw_input('Select an option: ')
            if not opt.isdigit(): continue
            opt = int(opt) - 1
            if opt in range(3): return opt

    def getSetting(self):
        print ""
        print "1: Change board display"
        print "2: Change board size"
        print "3: Change computer search depth"
        print "4: Exit settings"
        while(True):
            opt = raw_input('Select an setting: ')
            if not opt.isdigit(): continue
            opt = int(opt) - 1
            if opt in range(4): return opt

    def getDisplay(self):
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
        if again == "": return False
        again = again.upper()[0]
        return again == 'Y'
    
    def reallyQuit(self):
        quit = raw_input('Are you sure you want to quit? (Y/N): ')
        if quit == "": return False
        quit = quit.upper()[0]
        return quit == 'Y'

    def getChoiceUser(self, possibilities):
        count = 1
        for pos in possibilities:
            print str(count) + ":", \
            "(" + Enums.getAlpha(pos.x) + "," + str(pos.y+1) + ")"
            count = count + 1
        while(True):
            move = raw_input('Enter your move: ')
            if move == "": continue
            if move.upper()[0] == 'Q':
                if not self.reallyQuit(): continue
                else: return None
            if not move.isdigit(): continue
            choice = int(move)
            if choice != 0 and choice < len(possibilities)+1:
                return possibilities[choice-1]

    def getChoiceComputer(self, possibilities):
        print Color.str[self.ai] + " is thinking..."
        choice = Tree(self.board, self.ai).getBest(self.depth)
        print Color.str[self.ai] + " selects " + \
        "(" + Enums.getAlpha(possibilities[choice].x) + \
        "," + str(possibilities[choice].y+1) + ")"
        return possibilities[choice]

    def endTurn(self):
        print self.board
        # raw_input("Continue?")
        self.turn = Color.opp[self.turn]

game = Reversi()
game.start()
