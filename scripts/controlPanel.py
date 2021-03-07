import display, robot, math, time, VectorClass, simulator, controller
from Tkinter import *

import tf2_py
import tf2_ros
import rospy, os, time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import geometry_msgs.msg

import roslib; roslib.load_manifest('urdfdom_py')
from urdf_parser_py.urdf import URDF

class CoordinateControls(object):
    def __init__(self, root, controller, position, joints):

        self.root = root
        self.controller = controller
        self.position = position

        self.n = 0
        self.labels = []
        self.backgrounds = []
        self.indicators = []
        self.minusButtons = []
        self.plusButtons = []

        self.lowers = []
        self.uppers = []
        self.values = []
        self.targets = []

        for j in joints:
            self.addVisualiser(j)

    def addVisualiser(self, joint):
        p = self.position + Vector2(0,60)*self.n
        self.n += 1

        self.backgrounds.append(Canvas(self.root, width=200, height=20, bg="grey"))
        self.backgrounds[-1].place(x=p.x+20, y=p.y+20)

        self.labels.append(Label(self.root, text=joint[0]))
        self.labels[-1].place(x=p.x+100, y=p.y)

        self.labels.append(Label(self.root, text=str(joint[1])))
        self.labels[-1].place(x=p.x, y=p.y)
        self.lowers.append(joint[1])

        self.labels.append(Label(self.root, text=str(joint[2])))
        self.labels[-1].place(x=p.x + 220, y=p.y)
        self.uppers.append(joint[2])

        self.values.append(0)
        self.targets.append(0)

        index = self.n-1

        self.minusButtons.append(Button(self.root, text="-", command=lambda: self.moveTarget(-1, index)))
        self.minusButtons[-1].place(x=p.x, y=p.y+20)

        self.plusButtons.append(Button(self.root, text="+", command=lambda: self.moveTarget(+1, index)))
        self.plusButtons[-1].place(x=p.x+230, y=p.y+20)

    def moveTarget(self, x, i):
        delta = 10
        self.targets[i] += x*delta
        if(self.targets[i] < self.lowers[i]):
            self.targets[i] = self.lowers[i]
        elif(self.targets[i] > self.uppers[i]):
            self.targets[i] = self.uppers[i]
        self.controller.setJointState(self.getTargets())

    def setTargets(self, angles):
        for i in range(len(angles)):
            self.targets[i] = angles[i]

    def getTargets(self):
        v = []
        for i in self.targets:
            v.append(i)
        return v

    def setValues(self, values):
        for v in range(len(values)):
            self.values[v] = degrees(values[v])
            self.labels[v*3]["text"] = "J"+str(v)+" :- "+str(round(self.values[v],1))

    def update(self):
        for i in range(len(self.backgrounds)):
            self.backgrounds[i].delete(ALL)
            val = self.values[i]
            target = self.targets[i]
            lower = self.lowers[i]
            upper = self.uppers[i]
            pos = 200 * (val-lower)/(upper-lower)
            self.backgrounds[i].create_rectangle(pos-5,0,pos+5,20, fill = "red")
            pos = 200*(target-lower)/(upper-lower)
            self.backgrounds[i].create_rectangle(pos-5, 0, pos+5, 20, fill="green")
            self.backgrounds[i].update()



class ControlPanel():
    def __init__(self):
        self.root = Tk()
        self.root.geometry("600x600")

        self.solver = robot.Robot()
        self.preview = robot.Robot()

        self.controlBars = None
        self.horizontalDisplay = None
        self.effectorControls = None
        self.controller = None()

    def setup(self):
        pass

    def GetJoints(self):
        pass

    def Update():
        pass




if __name__ == '__main__':
    rospy.init_node('newController', anonymous=True)
    rospy.logerr("controller starting")
    rate = rospy.Rate(50)
    panel = ControlPanel()
    panel.setup()
    
    while not rospy.is_shutdown():
        panel.update()
        rate.sleep()