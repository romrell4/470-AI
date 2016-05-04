#!/bin/bash
import math

SIZE_OF_GRID = 100
# SIZE_OF_GRID = 10

class Node:
    def __init__(self, point, cost, path)
        self.point = point
        self.cost = cost
        self.path = path

class Queue:
    def __init(self):
        self.q = []

    def add(self, node):
        inList = False
        for index in range(len(self.q)):
            if node.cost.f < q[index].cost.f:
                q.insert(index, node)
                inList = True
        if not inList:
            q.append(node)

    def pop(self):
        return q.pop(0)

class Cost:
    def __init__(self, f, g):
        self.f = f
        self.g = g

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.accessiblePoints = []

    def inAprilTag(self):
    	return False

    def isAccessible(self, point):
    	return self != point and abs(self.x - point.x) <= SIZE_OF_GRID and abs(self.y - point.y) <= SIZE_OF_GRID

    def addAccessiblePoint(self, point):
    	self.accessiblePoints.append(point)

    def fullString(self):
    	result = str(self) + " -> "
    	for accessiblePoint in self.accessiblePoints:
    		result += str(accessiblePoint) + ", "
    	return result

    def __str__(self):
    	return "(" + str(self.x) + ", " + str(self.y) + ")"

def runFullAlgorithm():
	points = discretizeArea()
	for point in points:
		print point.fullString()
	doAStar()

def doAStar():
	print "Doing A*"

def discretizeArea():
	return createGridMap(getGridCenters(getLowerRightPoint()))

def getLowerRightPoint():
	#TODO: Fill in logic
	return Point(1500, 1400)

def getGridCenters(maxPoint):
	gridCenters = []
	for x in range(SIZE_OF_GRID/2, maxPoint.x, SIZE_OF_GRID):
		for y in range(SIZE_OF_GRID/2, maxPoint.y, SIZE_OF_GRID):
			point = Point(x, y)
			if not point.inAprilTag():
				gridCenters.append(point)

	# gridCenters = []
	# gridCenters = [Point(0, 0), Point(0, 10), Point(0, 20), Point(10, 0), Point(10, 10), Point(10, 20), Point(20, 0), Point(20, 10), Point(20, 20)]
	return gridCenters

def createGridMap(allPoints):
	for point in allPoints:
		for otherPoint in allPoints:
			if point.isAccessible(otherPoint):
				point.addAccessiblePoint(otherPoint)
	return allPoints


runFullAlgorithm()
