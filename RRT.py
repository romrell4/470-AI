#!/usr/bin/python

import random, math
#from World import World
from Node import Node
from Point import Point

# RRT class
class RRT:
    def __init__(self, start, end, delta, world):
        self.start = start
        self.end = end
        self.delta = delta
        self.world = world
        self.path = self.findPath()

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
                    startPath = startTree[len(startTree) - 1].path
                    endPath = endTree[len(endTree) - 1].path
                    endPath.reverse()
                    return startPath + endPath

            nearNode = self.getNearNode(randPoint, endTree)
            newPoint = self.getNewPoint(randPoint, nearNode.point)
            if newPoint != None:
                nearNode.point.addAccessiblePoint(newPoint)
                newPoint.addAccessiblePoint(nearNode.point)
                endTree.append(Node(newPoint, None, nearNode.path + [newPoint]))
                nearNode = self.getNearNode(newPoint, startTree)
                if nearNode.point.distanceTo(newPoint) <= self.delta:
                    #Link up the two tree paths and return
                    nearNode.point.addAccessiblePoint(newPoint)
                    newPoint.addAccessiblePoint(nearNode.point)
                    startPath = startTree[len(startTree) - 1].path
                    endPath = endTree[len(endTree) - 1].path
                    endPath.reverse()
                    return startPath + endPath

    def getRandPoint(self):
        #Return a new random point within the boundaries of the world
        bounds = self.world.getLowerRightPoint();
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
        atan = math.atan2(randPoint.y - nearPoint.y, randPoint.x - nearPoint.x)
        x = self.delta * math.cos(atan) + nearPoint.x
        y = self.delta * math.sin(atan) + nearPoint.y
        pt = Point(x, y)
        if self.world.inAprilTag(pt):
            return None
        return pt


