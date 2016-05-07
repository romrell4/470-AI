#!/usr/bin/python

from Node import Node

#Queue Class
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
