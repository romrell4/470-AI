#!/usr/bin/python

import math
from Point import Point
from Field import Field

# PolyField Class
class PolyField(Field):

    def __init__(self, id, poly):
    	Field.__init__(self, id)
        self.poly = poly

    def addPoint(self, x, y):
        self.poly.append(Point(x, y))

    def getCenter(self):
        count = x = y = 0
        for p in self.poly:
            count += 1
            x += int(p.x)
            y += int(p.y)
        x /= count
        y /= count
        return Point(x, y)

    def getDistanceFromCenter(self, point):
        c = self.getCenter()
        x = c.x - point.x
        y = c.y - point.y
        x *= x
        y *= y
        return math.sqrt(x + y)

    def getAngle(self, point):
        c = self.getCenter()
        x = c.x - point.x
        y = point.y - c.y
        return (x, y)

    def getAngleToCenter(self, point):
        (x, y) = self.getAngle(point)
        return math.atan2(y, x)

    def getAngleFromCenter(self, point):
        (x, y) = self.getAngle(point)
        return math.atan2(-y, -x)

    def getAngleTangentToCenter(self, point, clockwise):
        (x, y) = self.getAngle(point)
        if clockwise:
            return math.atan2(y, x) + math.pi/2 
        else: 
            return math.atan2(y, x) - math.pi/2

    def getCircleRadius(self):
        x = (self.poly[0].x + self.poly[1].x) / 2
        y = (self.poly[0].y + self.poly[1].y) / 2
        point = Point(x, y)
        return self.getDistanceFromCenter(point)

    def inCircle(self, point):
        if self.getDistanceFromCenter(point) < self.getCircleRadius():
            return True
        else:
            return False

    def inBound(self, point, dist):
        if self.getDistanceFromCenter(point) < self.getCircleRadius() + dist:
            return True
        else:
            return False

    def inSquare(self, point):
        intersections = 0
        for index in range(len(self.poly)):
            point1 = self.poly[index]
            point2 = None
            if (index + 1) in range(len(self.poly)):
                point2 = self.poly[index + 1]
            else:
                point2 = self.poly[0]
            if point.intersectsVertically(point1, point2):
                intersections += 1
        return intersections % 2 == 1

    def __str__(self):
        return "center=" + str(self.getCenter())
