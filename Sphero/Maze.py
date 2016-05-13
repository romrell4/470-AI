#!/usr/bin/python

from Point import Point

#Maze Class
class Maze:
    def __init__(self, walls, goals, step, x, y):
        self.grid = []
        self.discretize(step, x, y)
        self.buildWalls(walls)
        self.linkPoints()
        self.maze = self.flatten()
        self.goals = self.calcGoals(goals, step)
    
    def discretize(self, grid_size, x_max, y_max):
        for i in range(y_max + 1):
            #print "i=" + str(i) + " y_max=" + str(y_max)
            self.grid.append([])
            for j in range(x_max + 1):
                #print "j=" + str(j) + " x_max=" + str(x_max)
                self.grid[i].append(Point((i + 1) * grid_size, (j + 1) * grid_size))

    def buildWalls(self, walls):
        for i in range(len(walls)):
            self.grid[walls[i].y][walls[i].x] = None
    
    def linkPoints(self):
        y_max = len(self.grid) - 1
        for i in range(y_max):
            x_max = len(self.grid[i]) - 1
            for j in range(x_max):
                if self.grid[i][j] is None :
                    continue
                if i > 0 and not self.grid[i - 1][j] is None :
                    self.grid[i][j].addAccessiblePoint(self.grid[i - 1][j])
                if i < y_max and not self.grid[i + 1][j] is None :
                    self.grid[i][j].addAccessiblePoint(self.grid[i + 1][j])
                if j > 0 and not self.grid[i][j - 1] is None :
                    self.grid[i][j].addAccessiblePoint(self.grid[i][j - 1])
                if j < x_max and not self.grid[i][j + 1] is None :
                    self.grid[i][j].addAccessiblePoint(self.grid[i][j + 1])

                if i > 0 and j > 0 and not self.grid[i - 1][j - 1] is None :
                    self.grid[i][j].addAccessiblePoint(self.grid[i - 1][j - 1])
                if i > 0 and j < x_max and not self.grid[i - 1][j + 1] is None :
                    self.grid[i][j].addAccessiblePoint(self.grid[i - 1][j + 1])
                if i < y_max and j > 0 and not self.grid[i + 1][j - 1] is None :
                    self.grid[i][j].addAccessiblePoint(self.grid[i + 1][j - 1])
                if i < y_max and j < x_max and not self.grid[i + 1][j + 1] is None :
                    self.grid[i][j].addAccessiblePoint(self.grid[i + 1][j + 1])
            
    def flatten(self):
        points = []
        for i in range(len(self.grid)):
            self.grid[i][:] = [x for x in self.grid[i] if x is not None]
            points = points + self.grid[i]
        return points

    def calcGoals(self, goals, grid_size):
        points = []
        for i in range(len(goals)):
            coord = goals[i]
            points.append(Point((coord.x + 1) * grid_size, (coord.y + 1) * grid_size))
        return points



