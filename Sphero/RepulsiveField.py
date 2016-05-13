#!/usr/bin/python

from Point import Point
from Vector import Vector
from PolyField import PolyField

# RepulsiveField Class
class RepulsiveField(PolyField):

    def __init__(self, id, alpha, bound, poly):
        PolyField.__init__(self, id, poly)
        self.alpha = alpha
        self.bound = bound

    def getVect(self, point):
        if self.inCircle(point):
            return Vector(self.getAngleFromCenter(point), self.alpha)
        elif self.inBound(point, self.bound):
            velocity = self.alpha * ((self.bound + self.getCircleRadius() - self.getDistanceFromCenter(point)) / self.bound)
            return Vector(self.getAngleFromCenter(point), velocity)
        else:
            return Vector(0, 0)
