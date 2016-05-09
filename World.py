#!/usr/bin/python

from Point import Point
from AttractiveField import AttractiveField
from RepulsiveField import RepulsiveField
from CreativeField import CreativeField 
from TangentialField import TangentialField
from RandomField import RandomField
from BoxCanyonField import BoxCanyonField
from PathFinder import PathFinder

SIZE_OF_GRID = 70
SIZE_OF_FIELD_BOUND = SIZE_OF_GRID / 2

#World Class
class World:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.tags = []
        self.fields = []
        self.current = 0
        self.goals = [None, None, None, None, None, None]
        #self.start = Point(0, 0)
        #self.goal = Point(width, height)
        self.path = None

    def addTag(self, tag):
        self.tags.append(tag)

    def addField(self, field):
        self.fields.append(field)

    def createFields(self):
        for i in range(len(self.tags)):
            t_id = self.tags[i].id
            poly = self.tags[i].poly

            if t_id >= 0 and t_id <= 5:
                self.goals[t_id] = self.tags[i].getCenter()

            ''' 
            #Lab 1
            if t_id == 0:
                self.addField(AttractiveField(t_id, 100, 300, poly))
            elif t_id == 2:
                self.addField(TangentialField(t_id, 40, 40, 50, True, 100, poly))
            elif t_id == 3:
                self.addField(TangentialField(t_id, 40, 40, 50, False, 100, poly))
            else:
                self.addField(RepulsiveField(t_id, 50, 300, poly))
                #self.addField(TangentialField(t_id, 0, 0, 20, False, 150, poly))
                        
            #self.addField(RandomField(4, 40))
            self.addField(BoxCanyonField(5, 50, 100, 0, 0, self.width, self.height))
            '''

        self.goals[:] = [x for x in self.goals if x != None]

    def getVelocity(self, at):
        #Get the location of the sphero
        spheroPoint = Point(at.x, at.y)
        #Get the vector towards the goal.
        
        #path should be set by set by the pathfinder?
        if spheroPoint.distanceTo(self.path[self.current]) < SIZE_OF_GRID:
            if self.current >= len(self.path) - 1:
                self.current = 0
            else:
                self.current += 1
        dest = self.path[self.current]

        x = y = 0
        up = Point(dest.x, dest.y - 1)
        left = Point(dest.x - 1, dest.y)
        down = Point(dest.x, dest.y + 1)
        right = Point(dest.x + 1, dest.y)
        poly = [up, left, down, right]
        wayPoint = AttractiveField(-1, 80, 200, poly)
        tmpPoint = wayPoint.getVect(spheroPoint).getPoint()
        x += tmpPoint.x
        y += tmpPoint.y

        #print "fields: " + str(self.fields)
        for field in self.fields:
            if type(field) is AttractiveField and field.inCircle(spheroPoint):
                return Point(0, 0)
            tmpPoint = field.getVect(spheroPoint).getPoint()
            print "vector" + str(field.id) + "=" + str(tmpPoint)
            x += tmpPoint.x
            y += tmpPoint.y
        point = Point(x, y)
        #print "velocity= " + str(point)
        return point

    def inAprilTag(self, point):
        for tag in self.tags:
            if tag.id >= 0 and tag.id <= 5:
                continue
            #if tag.inSquare(point):
            #    return True
	    if tag.inBound(point, SIZE_OF_FIELD_BOUND):
		return True
        return False

    def getLowerRightPoint(self):
        return Point(self.width, self.height)

    def getGridCenters(self):
        gridCenters = []
        for x in range(SIZE_OF_GRID/2, self.width, SIZE_OF_GRID):
            for y in range(SIZE_OF_GRID/2, self.height, SIZE_OF_GRID):
                point = Point(x, y)
                if not self.inAprilTag(point):
                    gridCenters.append(point)
        return gridCenters

    def createGridMap(self, allPoints):
        for point in allPoints:
            for otherPoint in allPoints:
                if point.isAccessible(otherPoint, SIZE_OF_GRID):
                    point.addAccessiblePoint(otherPoint)
        return allPoints

    def discretizeArea(self):
        centers = self.getGridCenters()
        gridmap = self.createGridMap(centers)
        return gridmap





