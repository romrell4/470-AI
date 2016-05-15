from Square import Square
import Enums
from Enums import Color, Direction

class Board:
    def __init__(self, size):
        self.size = size
        self.max = size - 1
        self.weight = 0
        self.grid = []
        for i in range(size):
            self.grid.append([])
            for j in range(size):
                value = self.getSquareValue(i, j)
                self.weight += value
                self.grid[i].append(Square(i, j, value))
        for i in range(size):
            for j in range(size):
                if j > 0:
                    self.grid[i][j].neighbors[Direction.N] = self.grid[i][j - 1]
                if j < self.max:
                    self.grid[i][j].neighbors[Direction.S] = self.grid[i][j + 1]
                if i > 0:
                    self.grid[i][j].neighbors[Direction.W] = self.grid[i - 1][j]
                if i < self.max:
                    self.grid[i][j].neighbors[Direction.E] = self.grid[i + 1][j]
                if i > 0 and j > 0:
                    self.grid[i][j].neighbors[Direction.NW] = self.grid[i - 1][j - 1]
                if i < self.max and j > 0:
                    self.grid[i][j].neighbors[Direction.NE] = self.grid[i + 1][j - 1]
                if i > 0 and j < self.max:
                    self.grid[i][j].neighbors[Direction.SW] = self.grid[i - 1][j + 1]
                if i < self.max and j < self.max:
                    self.grid[i][j].neighbors[Direction.SE] = self.grid[i + 1][j + 1]
    
        m2 = size / 2
        m1 = m2 - 1
        self.grid[m1][m1].piece = Color.WHITE
        self.grid[m2][m2].piece = Color.WHITE
        self.grid[m1][m2].piece = Color.BLACK
        self.grid[m2][m1].piece = Color.BLACK

    def getPlayableSquares(self, color):
        #Add logic - return list of playable squares for a given color
        playableSquares = []
        for i in range(self.size):
            for j in range(self.size):
                if self.grid[i][j].isPlayable(color):
                    playableSquares.append(self.grid[i][j])
        return playableSquares

    def getConfig(self, x, y, color):
        board = Board(self.size)
        for i in range(self.size):
            for j in range(self.size):
                board.grid[i][j].piece = self.grid[i][j].piece
        board.play(x, y, color)
        return board

    def getScore(self, weighted):
        score = [0, 0, 0]
        if weighted:
            for i in range(self.size):
                for j in range(self.size):
                    square = self.grid[i][j]
                    score[square.piece] += square.value
        else:
            for i in range(self.size):
                for j in range(self.size):
                    score[self.grid[i][j].piece] += 1
        return score
    
    def getSquareValue(self, x, y):
        if (x == 0 and y == 0) or \
            (x == 0 and y == self.max) or \
            (x == self.max and y == 0) or \
            (x == self.max and y == self.max):
                return self.size
        elif (x == 0 and y == 1) or \
            (x == 1 and y == 0) or \
            (x == self.max - 1 and y == 0) or \
            (x == self.max and y == 1) or \
            (x == 0 and y == self.max - 1) or \
            (x == 1 and y == self.max) or \
            (x == self.max - 1 and y == self.max) or \
            (x == self.max and y == self.max - 1):
                return -self.size / 4
        elif x == 0 or y == 0 or x == self.max or y == self.max:
            return self.size / 2 - 1
        elif (x == 1 and y == 1) or \
            (x == self.max - 1 and y == 1) or \
            (x == 1 and y == self.max - 1) or \
            (x == self.max - 1 and y == self.max - 1):
                return 1
        elif x == 1 or y == 1 or x == self.max - 1 or y == self.max - 1:
            return self.size / 4
        else:
            return 1

    def play(self, x, y, color):
        self.grid[x][y].play(color)

    def __str__(self):
        return unicode(self).encode('utf-8')

    def __unicode__(self):
        result = "  "
        for j in range(self.size):
            result += Enums.getAlpha(j) + " "
        result += " \n"
        result += u'\u2554\u2550'
        for j in range(self.size):
            result += u'\u2550\u2550'
        result += u'\u2557'
        result += "\n"
        for j in range(self.size):
            result += u'\u2551' + " "
            for i in range(self.size):
                result += Color.chr[self.grid[i][j].piece] + " "
            result += u'\u2551' + " " + str(j+1) + "\n"
        result += u'\u255a\u2550'
        for j in range(self.size):
            result += u'\u2550\u2550'
        result += u'\u255d'
        score = self.getScore(False)
        result += "\n" + Color.str[Color.BLACK] + ": " + str(score[Color.BLACK])
        result += " " + Color.str[Color.WHITE] + ": " + str(score[Color.WHITE])

        return result + "\n"
