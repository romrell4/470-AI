#!/usr/bin/python

#Cost Class
class Cost:
    def __init__(self, f, g):
        self.f = f
        self.g = g

    def __str__(self):
        return "[f: " + str(self.f) + ", g: " + str(self.g) + "]"
