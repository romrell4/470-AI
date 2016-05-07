#!/usr/bin/python

from RRT import RRT
from Point import Point
from Queue import Queue
from Node import Node
from Cost import Cost

RRT_DELTA = 10

#PathFinder Class
class PathFinder:
    def __init__(self, world):
        aStarPath = self.doAStar(world)
        rrtPath = self.doRRT(world)
        world.path = aStarPath + rrtPath

    def doAStar(self, world):
        allPoints = world.discretizeArea()

	#Create queue with only root node
	q = Queue()
        begin = finish = None
        closest_start = closest_end = float("inf")
        for point in allPoints:
            dist_start = world.start.distanceTo(point)
            dist_end = world.goal.distanceTo(point)
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
	return bestNode.path + [world.goal]

    def doRRT(self, world):
        return RRT(world.goal, world.start, RRT_DELTA, world).path


