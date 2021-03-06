#!/bin/bash
import math
import random

SIZE_OF_GRID = 10
# SIZE_OF_GRID = 10

class Test:
    def testAStar1(self):
        points = []
        points.append(Point(0,0))
        points.append(Point(0,50))
        points.append(Point(10,0))
        points.append(Point(10,10))
        points.append(Point(10,30))
        points.append(Point(10,50))
        points.append(Point(20,0))
        points.append(Point(20,30))
        points.append(Point(20,50))
        points.append(Point(30,0))
        points.append(Point(30,10))
        points.append(Point(30,20))
        points.append(Point(30,30))
        points.append(Point(30,40))
        points.append(Point(30,50))
        points.append(Point(40,10))
        points.append(Point(40,20))
        points.append(Point(40,30))
        points.append(Point(50,20))

        path = doAStar(createGridMap(points))
        for point in path:
            print point

    def testAStar2(self):
        points = []
        points.append(Point(0,0))
        points.append(Point(10,50))
        points.append(Point(0,40))
        points.append(Point(10,0))
        points.append(Point(10,10))
        points.append(Point(10,30))
        points.append(Point(20,0))
        points.append(Point(20,30))
        points.append(Point(20,50))
        points.append(Point(30,0))
        points.append(Point(30,10))
        points.append(Point(30,20))
        points.append(Point(30,30))
        points.append(Point(30,40))
        points.append(Point(30,50))
        points.append(Point(40,10))
        points.append(Point(40,20))
        points.append(Point(40,30))
        points.append(Point(50,20))
        
        path = doAStar(createGridMap(points))
        for point in path:
            print point

    def testGetNewPoint(self):
        rrt = RRT(None, None, 6, None)
        nearPoint = Point(0, 0)
        print rrt.getNewPoint(Point(10, 5), nearPoint)
        print rrt.getNewPoint(Point(10, -5), nearPoint)
        print rrt.getNewPoint(Point(-10, 5), nearPoint)
        print rrt.getNewPoint(Point(-10, -5), nearPoint)

class Node:
    def __init__(self, point, cost, path):
        self.point = point
        self.cost = cost
        self.path = path

    def hasPointInPath(self, point):
    	return point in self.path

    def pathToString(self):
        result = "["
        first = True
        for point in self.path:
            if first:
                first = False
                result += str(point)
            else:
                result += ", "+str(point)

        result += "]"
        return result

    def __str__(self):
        return "[point: "+str(self.point)+", cost: "+str(self.cost)+", path: " + self.pathToString() + "]"

class Queue:
    def __init__(self):
        self.q = []

    def add(self, node):
        inList = False
        for index in range(len(self.q)):
            if node.cost.f < self.q[index].cost.f:
                self.q.insert(index, node)
                inList = True
                break
        if not inList:
            self.q.append(node)

    def pop(self):
        return self.q.pop(0)

    def __str__(self):
    	result = ""
    	for element in self.q:
    		result += "\n" + str(element.point) + " f=" + str(element.cost.f) + " path=" + str(element.pathToString())
    	return result

class Cost:
    def __init__(self, f, g):
        self.f = f
        self.g = g

    def __str__(self):
        return "[f: " + str(self.f) + ", g: " + str(self.g) + "]"

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

    def distanceTo(self, point):
        distX = self.x - point.x
        distY = self.y - point.y
        distX *= distX
        distY *= distY
        return math.sqrt(distX + distY)

    def fullString(self):
    	result = str(self) + " -> "
    	for accessiblePoint in self.accessiblePoints:
    		result += str(accessiblePoint) + ", "
    	return result

    def __str__(self):
    	return "(" + str(self.x) + ", " + str(self.y) + ")"

def runFullAlgorithm():
	points = discretizeArea()
	doAStar(points)

def doAStar(allPoints):
	#Create queue with only root node
	goalPoint = getGoal(allPoints)
	q = Queue()
	q.add(getRoot(allPoints, goalPoint))

	bestSoFar = float("inf")
	bestNode = None
	epochs = 0
	while(True):
		epochs+=1
		#print "Queue=" + str(q)
		parent = q.pop()
		#print "\nStarting loop with parent: " + str(parent)
		if (parent == None or parent.cost.f > bestSoFar):
			print "Either exhausted queue or best was found"
			break

		for child in parent.point.accessiblePoints:
			if parent.hasPointInPath(child):
				#print "Skipping child " + str(child)
				continue

			#print "Calculating child " + str(child)
			g = parent.cost.g + parent.point.distanceTo(child)
			h = child.distanceTo(goalPoint)
			#print "G=" + str(g) + ". H=" + str(h) + ". F=" + str(g+h)

			cost = Cost(g + h, g)
			node = Node(child, cost, parent.path + [parent.point])
			#print "Adding new node " + str(node)
			q.add(node)

			if (h == 0):
				print "We reached the end!"
				if (bestSoFar > g):
					print "Best so far getting updated from " + str(bestSoFar) + " to " + str(g)
					bestSoFar = g
					bestNode = node
					print "Best node is " + str(bestNode)

		#if epochs == 4:
		#	break

	if bestNode == None:
		return None
	return bestNode.path + [goalPoint]

def getRoot(allPoints, goalPoint):
	#TODO: Calculate closest point to Sphero
	point = allPoints[0]
	return Node(point, Cost(point.distanceTo(goalPoint), 0), [])

def getGoal(allPoints):
	return allPoints[1]

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

class RRT:
    def __init__(self, start, end, delta, world):
        self.start = start
        self.end = end
        self.delta = delta
        self.world = world
        #self.path = self.findPath()

    def findPath(self):
        startTree = [Node(self.start, None, [self.start])]
        endTree = [Node(self.end, None, [self.end])]
        #Build a rapidly exploring random tree
        while(True):
            randPoint = self.getRandPoint()

            nearNode = self.getNearNode(randPoint, startTree)
            newPoint = self.getNewPoint(randPoint, nearNode.point)
            if newPoint != None:
                nearNode.point.addAccessiblePoint(newPoint)
                newPoint.addAccessiblePoint(nearNode.point)
                startTree.append(Node(newPoint, None, nearNode.path + [newPoint]))
                nearNode = self.getNearNode(newPoint, endTree)
                if nearNode.point.distanceTo(newPoint) <= self.delta:
                    #Link up the two tree paths and return
                    nearNode.point.addAccessiblePoint(newPoint)
                    newPoint.addAccessiblePoint(nearNode.point)
                    return startTree.path + endTree.path.reverse

            nearNode = self.getNearNode(randPoint, endTree)
            newPoint = self.getNewPoint(randPoint, nearNode.point)
            if newPoint != None:
                nearNode.point.addAccessiblePoint(newPoint)
                newPoint.addAccessiblePoint(nearNode.point)
                endTree.append(Node(newPoint, None, nearNode.path + [newPoint]))
                nearNode = self.getNearNode(newPoint, startTree)
                if nearNode.distanceTo(newPoint) <= self.delta:
                    #Link up the two tree paths and return
                    nearNode.point.addAccessiblePoint(newPoint)
                    newPoint.addAccessiblePoint(nearNode.point)
                    return startTree.path + endTree.path.reverse

    def getRandPoint(self):
        #Return a new random point within the boundaries of the world
        bounds = getLowerRightPoint();
        x = random.random() * bounds.x;
        y = random.random() * bounds.y;
        return Point(x, y)

    def getNearNode(self, randPoint, treeSoFar):
        #Return the node in treeSoFar that is closest to randPoint
        closest = float("inf")
        for treeNode in treeSoFar:
            if randPoint.distanceTo(treeNode.point) < closest:
                closest = randPoint.distanceTo(treeNode.point)
                bestNode = treeNode
        return bestNode

    def getNewPoint(self, randPoint, nearPoint):
        #Return a new point that is distance self.delta from the near point in the direction of randPoint

        #TODO: If this new point falls inside an apriltag, return None
        atan = math.atan2(randPoint.y - nearPoint.y, randPoint.x - nearPoint.x)
        x = self.delta * math.cos(atan)
        y = self.delta * math.sin(atan)
        return Point(x, y)

#runFullAlgorithm()
#testAStar2()

test = Test()
test.testGetNewPoint()
