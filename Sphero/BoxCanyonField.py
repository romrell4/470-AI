#!/usr/bin/python

from Vector import Vector
from Field import Field

# BoxCanyonField Class
class BoxCanyonField(Field):
    
    def __init__(self, id, alpha, bound, origX, origY, width, height):
        Field.__init__(self, id)
        self.alpha = alpha
        self.bound = bound
        self.origX = origX
        self.origY = origY
        self.width = width
        self.height = height
    
    def getVect(self, point):
        onLeft = onRight = onTop = onBottom = False
        
        if point.x >= self.origX and point.x < self.origX + self.bound:
            onLeft = True
        if point.x > self.origX + self.width - self.bound and point.x <= self.origX + self.width:
            onRight = True
        if point.y >= self.origY and point.y < self.origY + self.bound:
            onTop = True
        if point.y > self.origX + self.height - self.bound and point.y <= self.origY + self.height:
            onBottom = True

        if onLeft and onTop:
            return Vector(-math.pi / 4, self.alpha * math.sqrt(2))
        elif onLeft and onBottom:
            return Vector(math.pi / 4, self.alpha * math.sqrt(2))
        elif onRight and onTop:
            return Vector(-3 * math.pi / 4, self.alpha * math.sqrt(2))
        elif onRight and onBottom:
            return Vector(3 * math.pi / 4, self.alpha * math.sqrt(2))
        elif onLeft:
            return Vector(0, self.alpha)
        elif onRight:
            return Vector(math.pi, self.alpha)
        elif onTop:
            return Vector(-math.pi / 2, self.alpha)
        elif onBottom:
            return Vector(math.pi / 2, self.alpha)
        else:
            return Vector(0, 0)
