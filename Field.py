#!/usr/bin/python

from Point import Point
from Vector import Vector

# Field Class
class Field:

    def __init__(self, id):
        self.id = id

    def getVect(self, point):
        raise NotImplementedError("Subclass must implement abstract method")

    def __str__(self):
        return "id=" + str(self.id)

