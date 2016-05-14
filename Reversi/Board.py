from Square import Square
import Enums
from Enums import Color, Direction, SIZE, MAX

M2 = SIZE / 2
M1 = M2 - 1

class Board:
    def __init__(self):
        self.grid = []
        for i in range(SIZE):
            self.grid.append([])
            for j in range(SIZE):
                self.grid[i].append(Square(i, j))
        for i in range(SIZE):
            for j in range(SIZE):
                if j > 0:
                    self.grid[i][j].neighbors[Direction.N] = self.grid[i][j - 1]
                if j < MAX:
                    self.grid[i][j].neighbors[Direction.S] = self.grid[i][j + 1]
                if i > 0:
                    self.grid[i][j].neighbors[Direction.W] = self.grid[i - 1][j]
                if i < MAX:
                    self.grid[i][j].neighbors[Direction.E] = self.grid[i + 1][j]
                if i > 0 and j > 0:
                    self.grid[i][j].neighbors[Direction.NW] = self.grid[i - 1][j - 1]
                if i < MAX and j > 0:
                    self.grid[i][j].neighbors[Direction.NE] = self.grid[i + 1][j - 1]
                if i > 0 and j < MAX:
                    self.grid[i][j].neighbors[Direction.SW] = self.grid[i - 1][j + 1]
                if i < MAX and j < MAX:
                    self.grid[i][j].neighbors[Direction.SE] = self.grid[i + 1][j + 1]
        self.grid[M1][M1].piece = Color.BLACK
        self.grid[M2][M2].piece = Color.BLACK
        self.grid[M1][M2].piece = Color.WHITE
        self.grid[M2][M1].piece = Color.WHITE

        #Config for testing Alpha Beta pruning
        '''
        self.grid[3][2].piece = Color.BLACK
        self.grid[3][3].piece = Color.BLACK
        self.grid[3][4].piece = Color.WHITE
        self.grid[3][5].piece = Color.BLACK
        self.grid[4][4].piece = Color.BLACK
        '''

    def getPlayableSquares(self, color):
        #Add logic - return list of playable squares for a given color
        playableSquares = []
        for i in range(SIZE):
            for j in range(SIZE):
                if self.grid[i][j].isPlayable(color):
                    playableSquares.append(self.grid[i][j])
        return playableSquares

    def getConfig(self, x, y, color):
        board = Board()
        for i in range(SIZE):
            for j in range(SIZE):
                board.grid[i][j].piece = self.grid[i][j].piece
        board.play(x, y, color)
        return board

    def getScore(self):
        score = [0, 0, 0]
        for i in range(SIZE):
            for j in range(SIZE):
                score[self.grid[i][j].piece] += Enums.getScore(i, j)
        return score

    def play(self, x, y, color):
        self.grid[x][y].play(color)

    def __str__(self):
        return unicode(self).encode('utf-8')

    def __unicode__(self):
        result = "  "
        for j in range(SIZE):
            result += Enums.getAlpha(j) + " "
        result += " \n"
        result += u'\u2554\u2550'
        for j in range(SIZE):
            result += u'\u2550\u2550'
        result += u'\u2557'
        result += "\n"
        for j in range(SIZE):
            result += u'\u2551' + " "
            for i in range(SIZE):
                piece = u'\u0f1d'
                color = self.grid[i][j].piece
                if color == Color.BLACK:
                    piece = u'\u25cf'
                if color == Color.WHITE:
                    piece = u'\u101d'
                result += piece + " "
            result += u'\u2551' + " " + str(j+1) + "\n"
        result += u'\u255a\u2550'
        for j in range(SIZE):
            result += u'\u2550\u2550'
        result += u'\u255d'
        score = self.getScore()
        result += "\nBlack: " + str(score[Color.BLACK])
        result += " White: " + str(score[Color.WHITE])

        return result + "\n"
