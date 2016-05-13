#!/usr/bin/python

from Vector import Vector
from Field import Field

# RandomField Class
class RandomField(Field):
    
    def __init__(self, id, alpha):
        Field.__init__(self, id)
        self.alpha = alpha

    def getVect(self, point):
        magnitude = self.alpha * random.random()
        direction = (2 * random.random() - 1) * math.pi
        return Vector(direction, magnitude)

