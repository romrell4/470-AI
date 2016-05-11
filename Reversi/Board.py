from Square import Square
from Enums import Direction

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
                if i > 0:
                    self.grid[i][j].neighbor[N] = self.grid[i - 1][j]
                if i < MAX:
                    self.grid[i][j].neighbor[S] = self.grid[i + 1][j]
                if j > 0:
                    self.grid[i][j].neighbor[W] = self.grid[i][j - 1]
                if j < MAX:
                    self.grid[i][j].neighbor[E] = self.grid[i][j + 1]
                if i > 0 and j > 0:
                    self.grid[i][j].neighbor[NW] = self.grid[i - 1][j - 1]
                if i > 0 and j < MAX:
                    self.grid[i][j].neighbor[NE] = self.grid[i - 1][j + 1]
                if i < MAX and j > 0:
                    self.grid[i][j].neighbor[SW] = self.grid[i + 1][j - 1]
                if i < MAX and j < MAX:
                    self.grid[i][j].neighbor[SE] = self.grid[i + 1][j + 1]
        self.grid[M1][M1].color = BLACK
        self.grid[M2][M2].color = BLACK
        self.grid[M1][M2].color = WHITE
        self.grid[M2][M1].color = WHITE

    def getPlayableSquares(self, color):
        #Add logic - return list of playable squares for a given color
        return []