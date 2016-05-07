#!/usr/bin/python

from RRT import RRT
from Point import Point
from Queue import Queue
from Node import Node
from Cost import Cost

RRT_DELTA = 25

#PathFinder Class
class PathFinder:
    def __init__(self, world):
        #firstPath = self.doAStar(world, world.start, world.goal)
	#secondPath = self.doAStar(world, world.goal, world.secondGoal)
	#thirdPath = self.doAStar(world, world.secondGoal, world.thirdGoal)
	#world.path = firstPath + secondPath + thirdPath
	aStarPath = self.doAStar(world, world.start, world.goal)
        print "aStar="
        for point in aStarPath:
            print str(point)
	
        rrtPath = self.doRRT(world, world.goal, world.start)
        print "rrt="
        for point in rrtPath:
            print str(point)
        world.path = aStarPath + rrtPath

    def doAStar(self, world, start, goal):
        allPoints = world.discretizeArea()

	#Create queue with only root node
	q = Queue()
        begin = finish = None
        closest_start = closest_end = float("inf")
        for point in allPoints:
            dist_start = start.distanceTo(point)
            dist_end = goal.distanceTo(point)
            if dist_start < closest_start:
                closest_start = dist_start
                begin = point
            if dist_end < closest_end:
                closest_end = dist_end
                finish = point
	q.add(Node(begin, Cost(point.distanceTo(finish), 0), [])) 

	bestSoFar = float("inf")
	bestNode = None
	while(True):
	    parent = q.pop()
	    print parent
	    if (parent == None or parent.cost.f > bestSoFar):
		break

	    for child in parent.point.accessiblePoints:
		if parent.hasPointInPath(child):
		    continue

		g = parent.cost.g + parent.point.distanceTo(child)
		h = child.distanceTo(finish)

		cost = Cost(g + h, g)
		node = Node(child, cost, parent.path + [parent.point])
		q.add(node)

		if (h == 0 and bestSoFar > g):
		    bestSoFar = g
		    bestNode = node


	if bestNode == None:
	    return None
	return bestNode.path + [goal]

    def doRRT(self, world, start, goal):
        return RRT(start, goal, RRT_DELTA, world).path


