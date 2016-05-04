#!/bin/bash

SIZE_OF_GRID = 100

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def inAprilTag(self):
    	return False

    def __str__(self):
    	return "(" + str(self.x) + ", " + str(self.y) + ")"

def discretizeArea():
	lowerRightPoint = getLowerRightPoint()
	gridCenters = getGridCenters(lowerRightPoint)
	for gridCenter in gridCenters:
		print gridCenter

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
	return gridCenters

discretizeArea()

