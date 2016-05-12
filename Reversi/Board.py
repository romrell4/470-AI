from Square import Square
from Enums import Color, Direction

SIZE = 8
MAX = SIZE - 1
M2 = SIZE / 2
M1 = M2 - 1

class Board:
    def __init__(self):
        self.grid = []
        for i in range(SIZE):
            self.grid.append([])
            for j in range(SIZE):
                self.grid[i].append(Square())
        for i in range(SIZE):
            for j in range(SIZE):
                if j > 0:
                    self.grid[i][j].neighbor[Direction.N] = self.grid[i][j - 1]
                if j < MAX:
                    self.grid[i][j].neighbor[Direction.S] = self.grid[i][j + 1]
                if i > 0:
                    self.grid[i][j].neighbor[Direction.W] = self.grid[i - 1][j]
                if i < MAX:
                    self.grid[i][j].neighbor[Direction.E] = self.grid[i + 1][j]
                if i > 0 and j > 0:
                    self.grid[i][j].neighbor[Direction.NW] = self.grid[i - 1][j - 1]
                if i < MAX and j > 0:
                    self.grid[i][j].neighbor[Direction.NE] = self.grid[i + 1][j - 1]
                if i > 0 and j < MAX:
                    self.grid[i][j].neighbor[Direction.SW] = self.grid[i - 1][j + 1]
                if i < MAX and j < MAX:
                    self.grid[i][j].neighbor[Direction.SE] = self.grid[i + 1][j + 1]
        self.grid[M1][M1].color = Color.BLACK
        self.grid[M2][M2].color = Color.BLACK
        self.grid[M1][M2].color = Color.WHITE
        self.grid[M2][M1].color = Color.WHITE

    def getPlayableSquares(self, color):
        #Add logic - return list of playable squares for a given color
        playableSquares = []
        for i in range(SIZE):
            for j in range(SIZE):
                if self.grid[i][j].isPlayable(color):
                    playableSquares.append(self.grid[i][j])
        return playableSquares

    def __str__(self):
        result = ""
        for j in range(SIZE + 2):
            result += "# "
        result += "\n"
        for j in range(SIZE):
            result += "# "
            for i in range(SIZE):
                result += str(self.grid[i][j].piece) + " "
            result += "#\n"
        for j in range(SIZE + 2):
            result += "# "
        return result

board = Board()
print board
playableSquares = board.getPlayableSquares(Color.BLACK)
for playableSquare in playableSquares:
    print playableSquare
# print board.grid[0][0].isPlayable(Color.BLACK)
# print board.grid[1][1].isPlayable(Color.BLACK)
# print board.grid[5][3].isPlayable(Color.BLACK)
# print board.grid[5][2].isPlayable(Color.BLACK)
