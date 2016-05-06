import AttractiveField, RepulsiveField, CreativeField, TangentialField, RandomField, BoxCanyonField

#World Class
class World:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.tags = []
        self.fields = []

    def addTag(self, tag):
        self.tags.append(tag):

    def addField(self, field):
        self.fields.append(field)

    def CreateFields(self):
        for i in range(len(self.tags)):
            t_id = self.tags[i].id
            poly = self.tags[i].poly

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

    def getVelocity(self, at):
        #Get the location of the sphero
        spheroPoint = Point(at.x, at.y)
        #Get the vector towards the goal.
        x = y = 0
        print "fields: " + str(self.fields)
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
