#!/usr/bin/python

import sys, rospy, math, random
from PyQt4 import QtGui, QtCore
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import ColorRGBA, Float32, Bool
from apriltags_intrude_detector.srv import apriltags_intrude
from apriltags_intrude_detector.srv import apriltags_info

# Point Class
class Point:

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
	return "x=" + str(self.x) + ", y=" + str(self.y)


# Vector Class
class Vector:

    def __init__(self, direction, magnitude):
        self.direction = direction
        self.magnitude = magnitude

    def getPoint(self):
        if self.direction > math.pi/2:
            x = -self.magnitude * math.cos(math.pi - self.direction)
            y = self.magnitude * math.sin(math.pi - self.direction)
        elif self.direction < -math.pi/2:
            x = -self.magnitude * math.cos(self.direction + math.pi)
            y = -self.magnitude * math.sin(self.direction + math.pi)
        else:
            x = self.magnitude * math.cos(self.direction)
            y = self.magnitude * math.sin(self.direction)
	
        return Point(x, y)

# Field Class
class Field:

    def __init__(self, id):
        self.id = id

    def getVect(self, point):
        raise NotImplementedError("Subclass must implement abstract method")

    def __str__(self):
        return "id=" + str(self.id)


# PolyField Class
class PolyField(Field):

    def __init__(self, id, poly):
    	Field.__init__(self, id)
        self.poly = poly

    def addPoint(self, x, y):
        self.poly.append(Point(x, y))

    def getCenter(self):
        count = x = y = 0
        for p in self.poly:
            count += 1
            x += int(p.x)
            y += int(p.y)
        x /= count
        y /= count
        return Point(x, y)

    def getDistanceFromCenter(self, point):
        c = self.getCenter()
        x = c.x - point.x
        y = c.y - point.y
        x *= x
        y *= y
        return math.sqrt(x + y)

    def getAngle(self, point):
        c = self.getCenter()
        x = c.x - point.x
        y = point.y - c.y
        return (x, y)

    def getAngleToCenter(self, point):
        (x, y) = self.getAngle(point)
        return math.atan2(y, x)

    def getAngleFromCenter(self, point):
        (x, y) = self.getAngle(point)
        return math.atan2(-y, -x)

    def getAngleTangentToCenter(self, point, clockwise):
        (x, y) = self.getAngle(point)
        if clockwise:
            return math.atan2(y, x) + math.pi/2 
        else: 
            return math.atan2(y, x) - math.pi/2

    def getCircleRadius(self):
        x = (self.poly[0].x + self.poly[1].x) / 2
        y = (self.poly[0].y + self.poly[1].y) / 2
        point = Point(x, y)
        return self.getDistanceFromCenter(point)

    def inCircle(self, point):
        if self.getDistanceFromCenter(point) < self.getCircleRadius():
            return True
        else:
            return False

    def inBound(self, point, dist):
        if self.getDistanceFromCenter(point) < self.getCircleRadius() + dist:
            return True
        else:
            return False

    def __str__(self):
        return "center=" + str(self.getCenter())


# AttractiveField Class
class AttractiveField(PolyField):

    def __init__(self, id, alpha, bound, poly):
        PolyField.__init__(self, id, poly)
        self.alpha = alpha
        self.bound = bound

    def getVect(self, point):
        if self.inCircle(point):
            return Vector(0, 0)
        elif self.inBound(point, self.bound):
            velocity = self.alpha * ((self.getDistanceFromCenter(point) - self.getCircleRadius()) / self.bound)
            return Vector(self.getAngleToCenter(point), velocity)
	else:
	    return Vector(self.getAngleToCenter(point), self.alpha)

# RepulsiveField Class
class RepulsiveField(PolyField):

    def __init__(self, id, alpha, bound, poly):
        PolyField.__init__(self, id, poly)
        self.alpha = alpha
        self.bound = bound

    def getVect(self, point):
        if self.inCircle(point):
            return Vector(self.getAngleFromCenter(point), self.alpha)
        elif self.inBound(point, self.bound):
            velocity = self.alpha * ((self.bound + self.getCircleRadius() - self.getDistanceFromCenter(point)) / self.bound)
            return Vector(self.getAngleFromCenter(point), velocity)
        else:
            return Vector(0, 0)

# CreativeField Class
class CreativeField(PolyField):
    
    def __init__(self, id, alpha, bound, poly):
        PolyField.__init__(self, id, poly)
        self.alpha = alpha
        self.bound = bound
    
    def getVect(self, point):
        if self.inCircle(point):
            return Vector(self.getAngleFromCenter(point), self.alpha)
        elif self.inBound(point, self.bound):
            velocity = self.alpha * ((self.bound + self.getCircleRadius() - self.getDistanceFromCenter(point)) / self.bound)
            return Vector(self.getAngleToCenter(point), velocity)
        else:
            return Vector(0, 0)

# TangentialField Class
class TangentialField(PolyField):

    def __init__(self, id, attract, repulse, alpha, clockwise, bound, poly):
        PolyField.__init__(self, id, poly)
        self.attract = attract
        self.repulse = repulse
        self.alpha = alpha
        self.clockwise = clockwise
        self.bound = bound

    def getVect(self, point):
        if self.inCircle(point):
            return Vector(self.getAngleFromCenter(point), self.repulse)
        elif self.inBound(point, self.bound):
            velocity = self.alpha * ((self.bound + self.getCircleRadius() - self.getDistanceFromCenter(point)) / self.bound) + 10
            return Vector(self.getAngleTangentToCenter(point, self.clockwise), velocity)
        else:
            return Vector(self.getAngleToCenter(point), self.attract)

# RandomField Class
class RandomField(Field):
    
    def __init__(self, id, alpha):
        Field.__init__(self, id)
        self.alpha = alpha

    def getVect(self, point):
        magnitude = self.alpha * random.random()
        direction = (2 * random.random() - 1) * math.pi
        return Vector(direction, magnitude)

# BoxCanyonField Class
class BoxCanyonField(Field):
    
    def __init__(self, id, alpha, bound, origX, origY, width, height):
        Field.__init__(self, id)
        self.alpha = alpha
        self.bound = bound
        self.origX = origX
        self.origY = origY
        self.width = width
        self.height = height
    
    def getVect(self, point):
        onLeft = onRight = onTop = onBottom = False
        
        if point.x >= self.origX and point.x < self.origX + self.bound:
            onLeft = True
        if point.x > self.origX + self.width - self.bound and point.x <= self.origX + self.width:
            onRight = True
        if point.y >= self.origY and point.y < self.origY + self.bound:
            onTop = True
        if point.y > self.origX + self.height - self.bound and point.y <= self.origY + self.height:
            onBottom = True

        if onLeft and onTop:
            return Vector(-math.pi / 4, self.alpha * math.sqrt(2))
        elif onLeft and onBottom:
            return Vector(math.pi / 4, self.alpha * math.sqrt(2))
        elif onRight and onTop:
            return Vector(-3 * math.pi / 4, self.alpha * math.sqrt(2))
        elif onRight and onBottom:
            return Vector(3 * math.pi / 4, self.alpha * math.sqrt(2))
        elif onLeft:
            return Vector(0, self.alpha)
        elif onRight:
            return Vector(math.pi, self.alpha)
        elif onTop:
            return Vector(-math.pi / 2, self.alpha)
        elif onBottom:
            return Vector(math.pi / 2, self.alpha)
        else:
            return Vector(0, 0)

# You implement this class
class Controller:
    stop = True # This is set to true when the stop button is pressed

    def __init__(self):
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.trackposSub = rospy.Subscriber("tracked_pos", Pose2D, self.trackposCallback)

    def trackposCallback(self, msg):
        # This function is continuously called
        if not self.stop:
            twist = Twist()
            # Change twist.linear.x to be your desired x velocity
    	    velocity = self.getVelocity(msg)
            twist.linear.x = velocity.x
            # Change twist.linear.y to be your desired y velocity
            twist.linear.y = velocity.y
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.cmdVelPub.publish(twist)

    def getVelocity(self, msg):
        #Get the location of the sphero
        spheroPoint = Point(msg.x, msg.y)
        #Get the vector towards the goal.
        x = y = 0
        for field in self.fields:
            if type(field) is AttractiveField and field.inCircle(spheroPoint):
                return Point(0, 0)
            tmpPoint = field.getVect(spheroPoint).getPoint()
            print "vector" + str(field.id) + "=" + str(tmpPoint)
            x += tmpPoint.x
            y += tmpPoint.y
        point = Point(x, y)
        return point

    def start(self):
        rospy.wait_for_service("apriltags_info")
        try:
            info_query = rospy.ServiceProxy("apriltags_info", apriltags_info)
            resp = info_query()

            self.fields = []
            for i in range(len(resp.polygons)):
                # A polygon (access points using poly.points)
                poly = resp.polygons[i]

                # The polygon's id (just an integer, 0 is goal, all else is bad)
                t_id = resp.ids[i]

                polyPoints = []
                for point in poly.points:
                    polyPoints.append(Point(point.x, point.y))

                if t_id == 0:
                    self.fields.append(AttractiveField(t_id, 100, 300, polyPoints))
                #elif t_id == 2:
                    #self.fields.append(TangentialField(t_id, 40, 60, 40, True, 150, polyPoints))
                elif t_id == 3:
                    self.fields.append(TangentialField(t_id, 40, 60, 40, False, 150, polyPoints))
                else:
                    self.fields.append(RepulsiveField(t_id, 50, 300, polyPoints))
                    #self.fields.append(TangentialField(t_id, 0, 0, 20, False, 150, polyPoints))
                        
            #self.fields.append(RandomField(4, 10))
            #self.fields.append(BoxCanyonField(5, 80, 100, 0, 0, 785, 575))

        except Exception, e:
            print "Exception: " + str(e)
        finally:
            self.stop = False

    def stop(self):
        self.stop = True


class SpheroIntrudeForm(QtGui.QWidget):
    controller = Controller()
    
    def __init__(self):
        super(QtGui.QWidget, self).__init__()
        self.resize(600, 480) 
        self.initUI()

        rospy.init_node('sphero_intrude', anonymous=True)
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cmdVelSub = rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback) 

        self.trackposSub = rospy.Subscriber("tracked_pos", Pose2D, self.trackposCallback) 
       
    def initUI(self):

        self.stateLabel = QtGui.QLabel("Position")
        self.stateTextbox = QtGui.QTextEdit()
        self.stateTextbox.setReadOnly(True)
        self.connect(self, QtCore.SIGNAL("sendPosIDText(PyQt_PyObject)"), self.updateStateTextbot)     
        
        key_instruct_label = """
	Control Your Sphero!
	---------------------------
	Moving around:
	   u    i    o
	   j    k    l
	   m    ,    .
	"""
        self.keyInstructLabel = QtGui.QLabel(key_instruct_label)
        self.cmdVelLabel = QtGui.QLabel("cmd_vel")
        self.cmdVelTextbox = QtGui.QTextEdit()
        self.cmdVelTextbox.setReadOnly(True)  
        self.connect(self, QtCore.SIGNAL("sendCmdVelText(PyQt_PyObject)"), self.updateCmdVelTextbox)

        self.aprilTagsInfoLabel = QtGui.QLabel("april tags info")
        self.aprilTagsInfoBtn = QtGui.QPushButton("Query")
        self.aprilTagsInfoBtn.clicked.connect(self.queryAprilTagsInfo)
        self.aprilTagsTextbox = QtGui.QTextEdit()
        self.aprilTagsTextbox.setReadOnly(True)
        self.connect(self, QtCore.SIGNAL("sendTagInfoText(PyQt_PyObject)"), self.updateAprilTagsTextbot)

        self.aprilTagsStartBtn = QtGui.QPushButton("Start")
        self.aprilTagsStartBtn.clicked.connect(self.controller.start)

        self.aprilTagsStopBtn = QtGui.QPushButton("Stop")
        self.aprilTagsStopBtn.clicked.connect(self.controller.stop)


        self.layout =  QtGui.QVBoxLayout()
        self.layout.addWidget(self.stateLabel)
        self.layout.addWidget(self.stateTextbox)
        self.layout.addWidget(self.keyInstructLabel)
        self.layout.addWidget(self.cmdVelLabel)
        self.layout.addWidget(self.cmdVelTextbox)
        hlayout = QtGui.QHBoxLayout()
        hlayout.addWidget(self.aprilTagsInfoLabel)
        hlayout.addWidget(self.aprilTagsInfoBtn)
        hlayout.addWidget(self.aprilTagsStartBtn)
        hlayout.addWidget(self.aprilTagsStopBtn)
        self.layout.addLayout(hlayout)
        self.layout.addWidget(self.aprilTagsTextbox)
        self.setLayout(self.layout)

        self.setWindowTitle("Sphero Intrude")
        self.show()

    def keyPressEvent(self, e): 
        twist = None       
        if e.key() == QtCore.Qt.Key_U:
            twist = Twist()
            twist.linear.x = -80; twist.linear.y = 80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_I:
            twist = Twist()  
            twist.linear.x = 0; twist.linear.y = 80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0     
        elif e.key() == QtCore.Qt.Key_O:
            twist = Twist()
            twist.linear.x = 80; twist.linear.y = 80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_J:
            twist = Twist()
            twist.linear.x = -80; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_K:
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_L:
            twist = Twist()
            twist.linear.x = 80; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_M:
            twist = Twist()
            twist.linear.x = -80; twist.linear.y = -80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_Comma:
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = -80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_Period:
            twist = Twist()
            twist.linear.x = 80; twist.linear.y = -80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0 
        if twist != None:
            self.cmdVelPub.publish(twist)

    def cmdVelCallback(self, msg):
        cmd_vel_text = "x=" + str(msg.linear.x) + " y=" + str(msg.linear.y)
        self.emit(QtCore.SIGNAL("sendCmdVelText(PyQt_PyObject)"), cmd_vel_text) 

    def updateCmdVelTextbox(self, value):
        self.cmdVelTextbox.moveCursor(QtGui.QTextCursor.End)
        self.cmdVelTextbox.ensureCursorVisible()
        self.cmdVelTextbox.append(str(value))
        self.cmdVelTextbox.update()

    def trackposCallback(self, msg):
        rospy.wait_for_service("apriltags_intrude")
        try:
            intrude_query = rospy.ServiceProxy("apriltags_intrude", apriltags_intrude)
            resp = intrude_query(int(msg.x), int(msg.y))
            pos_id_text = "["+str(int(msg.x))+"," +str(int(msg.y))+"]" + "(" + str(resp.id) + ")"
            self.emit(QtCore.SIGNAL("sendPosIDText(PyQt_PyObject)"), pos_id_text)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def updateStateTextbot(self, value):
        self.stateTextbox.moveCursor(QtGui.QTextCursor.End)
        self.stateTextbox.ensureCursorVisible()
        self.stateTextbox.append(str(value))
        self.stateTextbox.update()

    def queryAprilTagsInfo(self):
        #print "clicked"
        rospy.wait_for_service("apriltags_info")
        try:
            info_query = rospy.ServiceProxy("apriltags_info", apriltags_info)
            resp = info_query()
               
            #print str(resp)

            info_text = "" 
            for i in range(len(resp.polygons)):
                poly = resp.polygons[i]
                t_id = resp.ids[i]

                #print(str(poly))
                #print(str(t_id))
                info_text += "["+str(t_id)+"] "
                for p in poly.points:
                    info_text += "(" + str(int(p.x)) + "," + str(int(p.y)) + ")"
                info_text += "\n" 

            self.emit(QtCore.SIGNAL("sendTagInfoText(PyQt_PyObject)"), info_text)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def updateAprilTagsTextbot(self, value):
        self.aprilTagsTextbox.clear()
        self.aprilTagsTextbox.moveCursor(QtGui.QTextCursor.End)
        self.aprilTagsTextbox.ensureCursorVisible()
        self.aprilTagsTextbox.append(str(value))
        self.aprilTagsTextbox.update()        


if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    w = SpheroIntrudeForm()
    w.show()
    sys.exit(app.exec_())
  
        
