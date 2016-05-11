from Square import Square
from Enums import Color, Direction

SIZE = 8
WIDTH = SIZE
HEIGHT = SIZE
M2 = SIZE / 2
M1 = M2 - 1

class Board:
    def __init__(self):
        self.grid = []
        #Initialize grid
        for i in range(WIDTH):
            self.grid.append([])
            for j in range(HEIGHT):
                self.grid[i].append(Square(i, j))

        #Hook up neighbors
        for i in range(WIDTH):
            for j in range(HEIGHT):
                if i > 0:
                    self.grid[i][j].neighbors[Direction.N] = self.grid[i - 1][j]
                if i < WIDTH - 1:
                    self.grid[i][j].neighbors[Direction.S] = self.grid[i + 1][j]
                if j > 0:
                    self.grid[i][j].neighbors[Direction.W] = self.grid[i][j - 1]
                if j < HEIGHT - 1:
                    self.grid[i][j].neighbors[Direction.E] = self.grid[i][j + 1]
                if i > 0 and j > 0:
                    self.grid[i][j].neighbors[Direction.NW] = self.grid[i - 1][j - 1]
                if i > 0 and j < HEIGHT - 1:
                    self.grid[i][j].neighbors[Direction.NE] = self.grid[i - 1][j + 1]
                if i < WIDTH - 1 and j > 0:
                    self.grid[i][j].neighbors[Direction.SW] = self.grid[i + 1][j - 1]
                if i < WIDTH - 1 and j < HEIGHT - 1:
                    self.grid[i][j].neighbors[Direction.SE] = self.grid[i + 1][j + 1]
        self.grid[M1][M1].piece = Color.BLACK
        self.grid[M2][M2].piece = Color.BLACK
        self.grid[M1][M2].piece = Color.WHITE
        self.grid[M2][M1].piece = Color.WHITE

    def getPlayableSquares(self, color):
        #Add logic - return list of playable squares for a given color
        playableSquares = []
        for i in range(WIDTH):
            for j in range(HEIGHT):
                if self.grid[i][j].isPlayable(color):
                    playableSquares.append(self.grid[i][j])
        return playableSquares

    def __str__(self):
        result = ""
        for i in range(WIDTH + 2):
            result += "# "
        result += "\n"
        for i in range(WIDTH):
            result += "# "
            for j in range(HEIGHT):
                result += str(self.grid[i][j].piece) + " "
            result += "#\n"
        for i in range(WIDTH + 2):
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

