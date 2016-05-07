#!/usr/bin/python

import math

# Point Class
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.accessiblePoints = []

    def isAccessible(self, point, delta):
    	return self != point and abs(self.x - point.x) <= delta and abs(self.y - point.y) <= delta

    def addAccessiblePoint(self, point):
    	self.accessiblePoints.append(point)

    def distanceTo(self, point):
        distX = self.x - point.x
        distY = self.y - point.y
        distX *= distX
        distY *= distY
        return math.sqrt(distX + distY)

    def intersectsVertically(self, point1, point2):
        if self.x < point1.x and self.x < point2.x:
            return False
        if self.x > point1.x and self.x > point2.x:
            return False
        if point2.x - point1.x == 0:
            return False
        slope = (point2.y - point1.y) / (point2.x - point1.x)
        tempy = (self.x - point1.x) * slope + point1.y
        if tempy < self.y:
            return False
        else:
            return True

    def fullString(self):
    	result = str(self) + " -> "
    	for accessiblePoint in self.accessiblePoints:
    		result += str(accessiblePoint) + ", "
    	return result

    def __str__(self):
    	return "(" + str(self.x) + ", " + str(self.y) + ")"
