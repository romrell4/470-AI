#!/usr/bin/python

from RRT import RRT
from Point import Point
from Queue import Queue
from Node import Node
from Cost import Cost
from Maze import Maze

RRT_DELTA = 10

#PathFinder Class
class PathFinder:
    def __init__(self, world):
        #self.maze = self.Test1()
        self.points = world.discretizeArea() #self.maze.maze
        self.goals = world.goals #self.maze.goals

        world.path = []
        for index in range(len(self.goals) - 1):
            #world.path = world.path + self.doRRT(world, self.goals[index], self.goals[index + 1])
            world.path = world.path + self.doAStar(self.points, self.goals[index], self.goals[index + 1])
        if len(world.goals) >= 1:
            #world.path = world.path + self.doRRT(world, self.goals[len(world.goals) - 1], self.goals[0])
            world.path = world.path + self.doAStar(self.points, self.goals[len(world.goals) - 1], self.goals[0])

    def doAStar(self, gridMap, start, goal):
            print "AStar"

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

    def Test1(self):
        walls = []

        walls.append(Point(1,0))
        walls.append(Point(11,0))
        walls.append(Point(15,0))

        walls.append(Point(1,1))
        walls.append(Point(3,1))
        walls.append(Point(5,1))
        walls.append(Point(6,1))
        walls.append(Point(7,1))
        walls.append(Point(8,1))
        walls.append(Point(9,1))
        walls.append(Point(11,1))
        walls.append(Point(13,1))
        walls.append(Point(15,1))
        walls.append(Point(17,1))
        walls.append(Point(19,1))
        walls.append(Point(20,1))

        walls.append(Point(3,2))
        walls.append(Point(9,2))
        walls.append(Point(13,2))
        walls.append(Point(17,2))

        walls.append(Point(1,3))
        walls.append(Point(2,3))
        walls.append(Point(3,3))
        walls.append(Point(4,3))
        walls.append(Point(5,3))
        walls.append(Point(6,3))
        walls.append(Point(7,3))
        walls.append(Point(9,3))
        walls.append(Point(10,3))
        walls.append(Point(11,3))
        walls.append(Point(12,3))
        walls.append(Point(13,3))
        walls.append(Point(14,3))
        walls.append(Point(15,3))
        walls.append(Point(16,3))
        walls.append(Point(17,3))
        walls.append(Point(19,3))

        walls.append(Point(1,4))
        walls.append(Point(5,4))
        walls.append(Point(17,4))
        walls.append(Point(19,4))

        walls.append(Point(1,5))
        walls.append(Point(3,5))
        walls.append(Point(5,5))
        walls.append(Point(15,5))
        walls.append(Point(17,5))
        walls.append(Point(19,5))

        walls.append(Point(3,6))
        walls.append(Point(15,6))
        walls.append(Point(19,6))

        walls.append(Point(0,7))
        walls.append(Point(1,7))
        walls.append(Point(3,7))
        walls.append(Point(4,7))
        walls.append(Point(5,7))
        walls.append(Point(15,7))
        walls.append(Point(16,7))
        walls.append(Point(17,7))
        walls.append(Point(18,7))
        walls.append(Point(19,7))

        walls.append(Point(5,8))
        walls.append(Point(19,8))

        walls.append(Point(1,9))
        walls.append(Point(2,9))
        walls.append(Point(3,9))
        walls.append(Point(5,9))
        walls.append(Point(15,9))
        walls.append(Point(16,9))
        walls.append(Point(17,9))
        walls.append(Point(19,9))

        walls.append(Point(1,10))
        walls.append(Point(3,10))
        walls.append(Point(5,10))
        walls.append(Point(15,10))
        walls.append(Point(17,10))

        walls.append(Point(1,11))
        walls.append(Point(3,11))
        walls.append(Point(5,11))
        walls.append(Point(6,11))
        walls.append(Point(7,11))
        walls.append(Point(8,11))
        walls.append(Point(9,11))
        walls.append(Point(11,11))
        walls.append(Point(12,11))
        walls.append(Point(13,11))
        walls.append(Point(14,11))
        walls.append(Point(15,11))
        walls.append(Point(17,11))
        walls.append(Point(19,11))
        walls.append(Point(20,11))

        walls.append(Point(3,12))
        walls.append(Point(9,12))
        walls.append(Point(17,12))

        walls.append(Point(0,13))
        walls.append(Point(1,13))
        walls.append(Point(3,13))
        walls.append(Point(4,13))
        walls.append(Point(5,13))
        walls.append(Point(7,13))
        walls.append(Point(9,13))
        walls.append(Point(10,13))
        walls.append(Point(11,13))
        walls.append(Point(13,13))
        walls.append(Point(14,13))
        walls.append(Point(15,13))
        walls.append(Point(16,13))
        walls.append(Point(17,13))
        walls.append(Point(19,13))

        walls.append(Point(7,14))
        walls.append(Point(19,14))

        goals = []

        goals.append(Point(0,0))
        goals.append(Point(20,14))
        goals.append(Point(0,14))
        goals.append(Point(20,0))
        goals.append(Point(2,10))
        goals.append(Point(10,7))
        goals.append(Point(16,10))

        return Maze(walls, goals, 35, 20, 14)

