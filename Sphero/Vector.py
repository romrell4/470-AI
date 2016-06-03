#!/usr/bin/python

import math

# Vector Class
class Vector:

    def __init__(self, direction, magnitude):
        self.direction = direction
        self.magnitude = magnitude

    def calculateXandY(self):
        if self.direction > math.pi/2:
            x = -self.magnitude * math.cos(math.pi - self.direction)
            y = self.magnitude * math.sin(math.pi - self.direction)
        elif self.direction < -math.pi/2:
            x = -self.magnitude * math.cos(self.direction + math.pi)
            y = -self.magnitude * math.sin(self.direction + math.pi)
        else:
            x = self.magnitude * math.cos(self.direction)
            y = self.magnitude * math.sin(self.direction)
	
        return (x, y)
