import PolyField

# AttractiveField Class
class AttractiveField(PolyField):

    def __init__(self, id, alpha, bound, poly):
        PolyField.__init__(self, id, poly)
        self.alpha = alpha
        self.bound = bound

    def getVect(self, point):
        if self.inCircle(point):
            return Vector(0, 0)
        elif self.inBound(point, self.bound):
            velocity = self.alpha * ((self.getDistanceFromCenter(point) - self.getCircleRadius()) / self.bound)
            return Vector(self.getAngleToCenter(point), velocity)
	else:
	    return Vector(self.getAngleToCenter(point), self.alpha)
