#!/usr/bin/python

from Point import Point

#Maze Class
class Maze:
    def __init__(self):
        self.maze = []
        self.maze.append(self.Test1())
    
    def discretize(self, grid_size, x_max, y_max):
        coord = []
        for i in range(y_max):
            coord.append([])
            for j in range(x_max):
                coord[i].append(Point((i + 1) * grid_size), (j + 1) * grid_size))

        for i in range(y_max):
            for j in range(x_max):
                if i > 0:
                    coord[i][j].addAccessiblePoint(coord[i - 1][j])
                if i < y_max:
                    coord[i][j].addAccessiblePoint(coord[i + 1][j])
                if j > 0:
                    coord[i][j].addAccessiblePoint(coord[i][j - 1])
                if j < x_max:
                    coord[i][j].addAccessiblePoint(coord[i][j + 1])

                if i > 0 and j > 0:
                    coord[i][j].addAccessiblePoint(coord[i - 1][j - 1])
                if i > 0 and j < x_max:
                    coord[i][j].addAccessiblePoint(coord[i - 1][j + 1])
                if i < y_max and j > 0:
                    coord[i][j].addAccessiblePoint(coord[i + 1][j - 1])
                if i < y_max and j < x_max:
                    coord[i][j].addAccessiblePoint(coord[i + 1][j + 1])
        
        return coord
            
    def flatten(self, coord)
        points = []
        for i in range(len(coord)):
            points = points + coord[i]
        return points

    def Test1(self):
        coord = discretize(35, 20, 14)
        
        del coord[0][15]
        del coord[0][11]
        del coord[0][1]
        
        del coord[1][20]
        del coord[1][19]
        del coord[1][17]
        del coord[1][15]
        del coord[1][13]
        del coord[1][11]
        del coord[1][9]
        del coord[1][8]
        del coord[1][7]
        del coord[1][6]
        del coord[1][5]
        del coord[1][3]
        del coord[1][1]
        
        del coord[2][17]
        del coord[2][13]
        del coord[2][9]
        del coord[2][3]
        
        del coord[3][19]
        del coord[3][17]
        del coord[3][16]
        del coord[3][15]
        del coord[3][14]
        del coord[3][13]
        del coord[3][12]
        del coord[3][11]
        del coord[3][10]
        del coord[3][9]
        del coord[3][7]
        del coord[3][6]
        del coord[3][5]
        del coord[3][4]
        del coord[3][3]
        del coord[3][2]
        del coord[3][1]
        
        del coord[4][19]
        del coord[4][17]
        del coord[4][5]
        del coord[4][1]
        
        del coord[5][19]
        del coord[5][17]
        del coord[5][15]
        del coord[5][5]
        del coord[5][3]
        del coord[5][1]
        
        del coord[6][19]
        del coord[6][15]
        del coord[6][3]
        
        del coord[7][19]
        del coord[7][18]
        del coord[7][17]
        del coord[7][16]
        del coord[7][15]
        del coord[7][5]
        del coord[7][4]
        del coord[7][3]
        del coord[7][1]
        del coord[7][0]
        
        del coord[8][19]
        del coord[8][5]
        
        del coord[9][19]
        del coord[9][17]
        del coord[9][16]
        del coord[9][15]
        del coord[9][5]
        del coord[9][3]
        del coord[9][2]
        del coord[9][1]
        
        del coord[10][17]
        del coord[10][15]
        del coord[10][5]
        del coord[10][3]
        del coord[10][1]
        
        del coord[11][20]
        del coord[11][19]
        del coord[11][17]
        del coord[11][15]
        del coord[11][14]
        del coord[11][13]
        del coord[11][12]
        del coord[11][11]
        del coord[11][9]
        del coord[11][8]
        del coord[11][7]
        del coord[11][6]
        del coord[11][5]
        del coord[11][3]
        del coord[11][1]
        
        del coord[12][17]
        del coord[12][9]
        del coord[12][3]
        
        del coord[13][19]
        del coord[13][17]
        del coord[13][16]
        del coord[13][15]
        del coord[13][14]
        del coord[13][13]
        del coord[13][11]
        del coord[13][10]
        del coord[13][9]
        del coord[13][7]
        del coord[13][5]
        del coord[13][4]
        del coord[13][3]
        del coord[13][1]
        del coord[13][0]
        
        del coord[14][19]
        del coord[14][7]
        
        return self.flatten(coord)
