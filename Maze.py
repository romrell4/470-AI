#!/usr/bin/python

from Point import Point

#Maze Class
class Maze:
    def __init__(self):
        self.maze = []
        self.maze.append(self.Test1())
    
    def discretize(self, grid_size, x_max, y_max):
        grid = []
        for i in range(y_max + 1):
            #print "i=" + str(i) + " y_max=" + str(y_max)
            grid.append([])
            for j in range(x_max + 1):
                #print "j=" + str(j) + " x_max=" + str(x_max)
                grid[i].append(Point((i + 1) * grid_size, (j + 1) * grid_size))
        return grid
    
    def linkPoints(self, grid):
        y_max = len(grid) - 1
        for i in range(y_max):
            x_max = len(grid[i]) - 1
            for j in range(x_max):
                if grid[i][j] is None :
                    continue
                if i > 0 and not grid[i - 1][j] is None :
                    grid[i][j].addAccessiblePoint(grid[i - 1][j])
                if i < y_max and not grid[i + 1][j] is None :
                    grid[i][j].addAccessiblePoint(grid[i + 1][j])
                if j > 0 and not grid[i][j - 1] is None :
                    grid[i][j].addAccessiblePoint(grid[i][j - 1])
                if j < x_max and not grid[i][j + 1] is None :
                    grid[i][j].addAccessiblePoint(grid[i][j + 1])

                if i > 0 and j > 0 and not grid[i - 1][j - 1] is None :
                    grid[i][j].addAccessiblePoint(grid[i - 1][j - 1])
                if i > 0 and j < x_max and not grid[i - 1][j + 1] is None :
                    grid[i][j].addAccessiblePoint(grid[i - 1][j + 1])
                if i < y_max and j > 0 and not grid[i + 1][j - 1] is None :
                    grid[i][j].addAccessiblePoint(grid[i + 1][j - 1])
                if i < y_max and j < x_max and not grid[i + 1][j + 1] is None :
                    grid[i][j].addAccessiblePoint(grid[i + 1][j + 1])
        return grid
            
    def flatten(self, grid):
        points = []
        for i in range(len(grid)):
            grid[i][:] = [x for x in grid[i] if x is None]
            points = points + grid[i]
        return points

    def Test1(self):
        grid = self.discretize(35, 20, 14)
        
        grid[0][15] = None
        grid[0][11] = None
        grid[0][1] = None
        
        grid[1][20] = None
        grid[1][19] = None
        grid[1][17] = None
        grid[1][15] = None
        grid[1][13] = None
        grid[1][11] = None
        grid[1][9] = None
        grid[1][8] = None
        grid[1][7] = None
        grid[1][6] = None
        grid[1][5] = None
        grid[1][3] = None
        grid[1][1] = None
        
        grid[2][17] = None
        grid[2][13] = None
        grid[2][9] = None
        grid[2][3] = None
        
        grid[3][19] = None
        grid[3][17] = None
        grid[3][16] = None
        grid[3][15] = None
        grid[3][14] = None
        grid[3][13] = None
        grid[3][12] = None
        grid[3][11] = None
        grid[3][10] = None
        grid[3][9] = None
        grid[3][7] = None
        grid[3][6] = None
        grid[3][5] = None
        grid[3][4] = None
        grid[3][3] = None
        grid[3][2] = None
        grid[3][1] = None
        
        grid[4][19] = None
        grid[4][17] = None
        grid[4][5] = None
        grid[4][1] = None
        
        grid[5][19] = None
        grid[5][17] = None
        grid[5][15] = None
        grid[5][5] = None
        grid[5][3] = None
        grid[5][1] = None
        
        grid[6][19] = None
        grid[6][15] = None
        grid[6][3] = None
        
        grid[7][19] = None
        grid[7][18] = None
        grid[7][17] = None
        grid[7][16] = None
        grid[7][15] = None
        grid[7][5] = None
        grid[7][4] = None
        grid[7][3] = None
        grid[7][1] = None
        grid[7][0] = None
        
        grid[8][19] = None
        grid[8][5] = None
        
        grid[9][19] = None
        grid[9][17] = None
        grid[9][16] = None
        grid[9][15] = None
        grid[9][5] = None
        grid[9][3] = None
        grid[9][2] = None
        grid[9][1] = None
        
        grid[10][17] = None
        grid[10][15] = None
        grid[10][5] = None
        grid[10][3] = None
        grid[10][1] = None
        
        grid[11][20] = None
        grid[11][19] = None
        grid[11][17] = None
        grid[11][15] = None
        grid[11][14] = None
        grid[11][13] = None
        grid[11][12] = None
        grid[11][11] = None
        grid[11][9] = None
        grid[11][8] = None
        grid[11][7] = None
        grid[11][6] = None
        grid[11][5] = None
        grid[11][3] = None
        grid[11][1] = None
        
        grid[12][17] = None
        grid[12][9] = None
        grid[12][3] = None
        
        grid[13][19] = None
        grid[13][17] = None
        grid[13][16] = None
        grid[13][15] = None
        grid[13][14] = None
        grid[13][13] = None
        grid[13][11] = None
        grid[13][10] = None
        grid[13][9] = None
        grid[13][7] = None
        grid[13][5] = None
        grid[13][4] = None
        grid[13][3] = None
        grid[13][1] = None
        grid[13][0] = None
        
        grid[14][19] = None
        grid[14][7] = None
        
        return self.flatten(self.linkPoints(grid))
