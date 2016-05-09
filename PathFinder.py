#!/usr/bin/python

from RRT import RRT
from Point import Point
from Queue import Queue
from Node import Node
from Cost import Cost

RRT_DELTA = 10

#PathFinder Class
class PathFinder:
    def __init__(self, world, discretization):
        world.path = []
        for index in range(len(goals) - 1):
            world.path = world.path + self.doAStar(discretization, world.goals[index], world.goals[index + 1])
        world.path = world.path + doRRT(world, world.goals[len(world.goals) - 1], world.goals[0])
        
        '''
        aStarPath = self.doAStar(world.discretizeArea(), world.start, world.goal)
        rrtPath = self.doRRT(world, world.goal, world.start)
        world.path = aStarPath + rrtPath
        '''

    def doAStar(self, gridMap, start, goal):

	    #Create queue with only root node
	    q = Queue()
            begin = finish = None
            closest_start = closest_end = float("inf")
            for point in gridMap:
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


