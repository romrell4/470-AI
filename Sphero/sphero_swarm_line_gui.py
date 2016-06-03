#!/usr/bin/python

import sys, rospy, math, time
from Vector import Vector
from PyQt4 import QtGui, QtCore
from DirectionCalculator import *

from sphero_swarm_node.msg import SpheroTwist, SpheroColor
from multi_apriltags_tracker.msg import april_tag_pos
STEP_LENGTH = 50
PREY_SIMPLE, PREY_TANGENTIAL, PREY_CHOICE, PREY_SMART = range(4)
PRED_DUMB, PRED_SMART = range(2)
PREY_MODE = PREY_CHOICE
PRED_MODE = PRED_DUMB

class SpheroSwarmLineForm(QtGui.QWidget):
    
    def __init__(self):
        super(QtGui.QWidget, self).__init__()
        self.PRED_KEY = 9
        self.PREY_KEY = 0
        self.speed = {0:90, 9:160}
        self.delay = 0
        self.resize(600, 480) 
        self.sphero_dict = {}
        self.initUI()
        self.initialized = False
        '''The Sphero bluetooth controller maps string names to addresses, The camera maps num to locations numToSphero
        and spheroToNum are dictoinaries that will map back and forth'''
        self.numToSphero = {}
        self.spheroToNum = {}
        self.order = [] #used to keep a follow the leadrer order
        self.location = {} #dictionary that maps sphero id nums to last known location
	self.preyLastLocations = [None, None, None, None, None, None, None, None]
	self.cycles = 0

        rospy.init_node('sphero_swarm_line_gui', anonymous=True)

        self.cmdVelPub = rospy.Publisher('cmd_vel', SpheroTwist, queue_size=1) #self.cmdVelPub is who we tell about to move sphero
        self.cmdVelSub = rospy.Subscriber("cmd_vel", SpheroTwist, self.cmdVelCallback)

        self.colorPub = rospy.Publisher('set_color', SpheroColor, queue_size=1) #who we tell if we want to update the color
        self.aprtSub = rospy.Subscriber('april_tag_pos', april_tag_pos, self.aprtCallback)
        #aprtSub tells us when april tags are updated. When this happens the callback function is called.
       
    def initUI(self):   
        
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

        self.spheroLabel = QtGui.QLabel("Spheros:")
        self.spheroListWidget = QtGui.QListWidget()
        self.refreshBtn = QtGui.QPushButton("Refresh")
        self.refreshBtn.clicked.connect(self.refreshDevices)
        btnGridLayout = QtGui.QGridLayout()
        btnGridLayout.addWidget(self.refreshBtn, 0, 4)

        layout =  QtGui.QVBoxLayout()
        layout.addWidget(self.keyInstructLabel)
        layout.addWidget(self.cmdVelLabel)
        layout.addWidget(self.cmdVelTextbox)
        layout.addWidget(self.spheroLabel)
        layout.addWidget(self.spheroListWidget)
        layout.addLayout(btnGridLayout)
        self.setLayout(layout)

        self.setWindowTitle("Sphero Swarm Teleop")
        self.show()

    def keyPressEvent(self, e): 
        twist = None 

        print "key pressed"   
        selected_items = self.spheroListWidget.selectedItems()
        if len(selected_items) == 0:
            return

        print "selected"
           
        if e.key() == QtCore.Qt.Key_U:
            twist = SpheroTwist() 
            twist.linear.x = -STEP_LENGTH; twist.linear.y = STEP_LENGTH; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_I:
            twist = SpheroTwist()  
            twist.linear.x = 0; twist.linear.y = STEP_LENGTH; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0     
        elif e.key() == QtCore.Qt.Key_O:
            twist = SpheroTwist()
            twist.linear.x = STEP_LENGTH; twist.linear.y = STEP_LENGTH; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_J:
            twist = SpheroTwist()
            twist.linear.x = -STEP_LENGTH; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_K:
            twist = SpheroTwist()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_L:
            twist = SpheroTwist()
            twist.linear.x = STEP_LENGTH; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_M:
            twist = SpheroTwist()
            twist.linear.x = -STEP_LENGTH; twist.linear.y = -STEP_LENGTH; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_Comma:
            twist = SpheroTwist()
            twist.linear.x = 0; twist.linear.y = -STEP_LENGTH; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_Period:
            twist = SpheroTwist()
            twist.linear.x = STEP_LENGTH; twist.linear.y = -STEP_LENGTH; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0 

        if twist != None:
            twist.name = str(selected_items[0].text())
            self.cmdVelPub.publish(twist)

    def cmdVelCallback(self, msg):
        cmd_vel_text = "(" + str(msg.name) + "),x=" + str(msg.linear.x) + " y=" + str(msg.linear.y)
        self.emit(QtCore.SIGNAL("sendCmdVelText(PyQt_PyObject)"), cmd_vel_text) 

    def updateCmdVelTextbox(self, value):
        self.cmdVelTextbox.moveCursor(QtGui.QTextCursor.End)
        self.cmdVelTextbox.ensureCursorVisible()
        self.cmdVelTextbox.append(str(value))
        self.cmdVelTextbox.update()

    ### called when refreshDevices is clicked.
    def refreshDevices(self):
        self.initialized = False
        self.spheroListWidget.clear()
        self.sphero_dict = rospy.get_param('/sphero_swarm/connected')
        self.numToSphero = {}
        self.spheroToNum = {}
        self.order = list()
        self.location = {}
        print(self.sphero_dict)

        for name in self.sphero_dict:
            num, ok = QtGui.QInputDialog.getInt(self, "Sphero num input", "Enter April Tag number for %s:" % name)
            self.numToSphero[num] = name
            self.spheroToNum[name] = name
            self.order[len(self.order):] = [num]
            self.location[num] = (-1,-1)
            self.spheroListWidget.addItem(name)
        self.spheroListWidget.setCurrentRow(0)
        self.initialized = True
        self.update()

    ### main body of algorithm should go here. MSG contains an id, x,y and orientation deta members
    def aprtCallback(self, msg):
        if not self.initialized: #still initializing
            return

        for key in self.location:
            self.location[key] = (-1,-1)

        for i in range(0,len(msg.id)):
            self.location[msg.id[i]] = (msg.pose[i].x, msg.pose[i].y)
            #print str(msg.id[i]) + " " + str(self.location[msg.id[i]])

        x = self.location[self.PREY_KEY][0] - self.location[self.PRED_KEY][0]
        y = self.location[self.PREY_KEY][1] - self.location[self.PRED_KEY][1]
        if x * x + y * y < 3025 and self.delay == 0:
            TEMP_KEY = self.PRED_KEY
            self.PRED_KEY = self.PREY_KEY
            self.PREY_KEY = TEMP_KEY
            self.delay = 8

        for key in msg.id:
            twist = SpheroTwist()
            twist.name = self.numToSphero[key]
            (twist.linear.x, twist.linear.y) = self.getVelocityForSphero(key)
            self.cmdVelPub.publish(twist) # how to tell sphero to move. all fields in twist must be explicitly set.


    def getVelocityForSphero(self, key):
	self.predLocation = self.location[self.PRED_KEY]
	self.preyLocation = self.location[self.PREY_KEY]
	
        angle = 0
        mag = 70

        if key == self.PRED_KEY:
            if self.delay > 0: 
                mag = 0
                self.delay -= 1
            else: mag = self.speed[key]
	    x = self.preyLocation[0] - self.predLocation[0]
            y = self.preyLocation[1] - self.predLocation[1]

	    if PRED_MODE == PRED_DUMB:
		angle = math.atan2(y, -x)
	    elif PRED_MODE == PRED_SMART:		
		preyLastLocation = self.preyLastLocations[(self.cycles + 1) % 8]

		#print preyLastLocation
		#print self.preyLocation
		#print

		if preyLastLocation is None:
		     preyLastLocation = self.preyLocation

		x += self.preyLocation[0] - preyLastLocation[0]
		y += self.preyLocation[1] - preyLastLocation[1]

		#print "Last Prey Location: " + str(self.preyLastLocation)
		#print "Current Prey Location: " + str(self.preyLocation)
		#print "Future Prey Location: " + str((x + self.predLocation[0], y + self.predLocation[1]))

		angle = math.atan2(y, -x)
	    
	else:
            mag = self.speed[key]
	    self.cycles += 1
	
	    self.preyLastLocations[self.cycles % 8] = self.preyLocation

	    x = self.predLocation[0] - self.preyLocation[0]
            y = self.predLocation[1] - self.preyLocation[1]

	    self.preyLastLocation = self.preyLocation

            if PREY_MODE == PREY_SIMPLE:
	    	angle = math.atan2(-y, x)
            elif PREY_MODE == PREY_TANGENTIAL:
                angle = math.atan2(-y, x) + math.pi/2
            elif PREY_MODE == PREY_CHOICE:
                if self.delay > 0:
                    angle = math.atan2(-y, x)
                else:
                    location = Point(self.preyLocation[0], self.preyLocation[1])
                    options = [math.atan2(-y, x) + math.pi/2, math.atan2(-y, x) - math.pi/2]
                    angle = getOptimalDirection(location, options)

	return Vector(angle, mag).calculateXandY()





if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    w = SpheroSwarmLineForm()
    w.show()
    sys.exit(app.exec_())
  
        
