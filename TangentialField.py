import PolyField

# TangentialField Class
class TangentialField(PolyField):

    def __init__(self, id, attract, repulse, alpha, clockwise, bound, poly):
        PolyField.__init__(self, id, poly)
        self.attract = attract
        self.repulse = repulse
        self.alpha = alpha
        self.clockwise = clockwise
        self.bound = bound

    def getVect(self, point):
        if self.inCircle(point):
            velocity = self.repulse * (self.getCircleRadius() - self.getDistanceFromCenter(point)) / self.bound
            return Vector(self.getAngleFromCenter(point), velocity)
        elif self.inBound(point, self.bound):
            velocity = self.alpha * ((self.bound + self.getCircleRadius() - self.getDistanceFromCenter(point)) / self.bound) + 10
            return Vector(self.getAngleTangentToCenter(point, self.clockwise), self.alpha)
        elif self.inBound(point, self.bound * 2):
            velocity = self.attract * ((self.bound * 2 + self.getCircleRadius() - self.getDistanceFromCenter(point)) / self.bound)
            return Vector(self.getAngleToCenter(point), velocity)
        else:
            return Vector(0, 0)
