#!/usr/bin/python

from Point import Point
from Cost import Cost

#Node Class
class Node:
    def __init__(self, point, cost, path):
        self.point = point
        self.cost = cost
        self.path = path

    def hasPointInPath(self, point):
    	return point in self.path

    def pathToString(self):
        result = "["
        first = True
        for point in self.path:
            if first:
                first = False
                result += str(point)
            else:
                result += ", "+str(point)

        result += "]"
        return result

    def __str__(self):
        return "[point: "+str(self.point)+", cost: "+str(self.cost)+", path: " + self.pathToString() + "]"
