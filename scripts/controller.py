#!/usr/bin/env python
from Tkinter import *
from VectorClass import Vector2, Vector3
from math import *
from display import Display
import robot

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import std_msgs.msg


class RadialDisplay(object):
    def __init__(self, root, controller, position):
        self.root = root
        self.controller = controller
        self.position = position
        self.angle = 0

        self.canvas = Canvas(self.root, height=80, width=80, bg="white")
        self.canvas.place(x=position.x, y=position.y+30)

        self.label = Label(self.root, text="Effector Angle")
        self.label.place(x=position.x, y=position.y)

        self.input = Entry(self.root, width=8)
        self.input.place(x=position.x+10, y=position.y+130)

        self.button = Button(self.root, text="Set", command=self.setAngleFromEntry)
        self.button.place(x=position.x+70, y=position.y+130)

        self.canvas.bind("<Button-1>", self.setAngleFromDial)

    def update(self):
        self.canvas.delete(ALL)
        self.canvas.create_oval(5,5,75,75, fill="grey")
        self.canvas.create_line(40,40,40+40*cos(radians(self.angle)), 40+40*sin(radians(self.angle)))
        self.canvas.update()



    def setAngleFromDial(self, event):
        offsetX = event.x-40
        offsetY = event.y-40
        angle = atan2(offsetY, offsetX)
        self.angle = degrees(angle)
        self.controller.setEffectorAngle(-angle)
        self.input.delete(0,END)
        self.input.insert(0, str(round(degrees(angle),1)))

    def setAngleFromEntry(self):
        self.angle = float(self.input.get())
        self.controller.setEffectorAngle(-radians(self.angle))


class PositionDisplay(object):
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


class Controller(object):
    #def __init__(self, joints, planner):
    def __init__(self, sim):
        self.sim = sim

        self.solver = robot.Robot()
        self.preview = robot.Robot()

        self.root = Tk()
        self.root.geometry("600x600")
        self.coords = []
        self.widgets = []
        self.controlBars = None

        self.visualiser = Display(self.root, h=300,w=600, pos=Vector2(0,300))
        self.visualiser.canvas.bind("<Button-1>", self.goToClick)
        self.visualiser.setScreenOrigin(0.25, 0.8)

        self.angleControl = RadialDisplay(self.root, self, Vector2(300,10))
        self.addWidgets()
        self.setEffectorAngle(0.0)

    def addWidgets(self):
        joints = self.sim.getJoints()

        for i in range(0,len(joints)):
            j = joints[i]
            length = self.sim.getJointLength(joints[i][0])
            self.solver.addLink(length, [j[1],j[2]])
            self.preview.addLink(length, [j[1],j[2]])

        self.controlBars = PositionDisplay(self.root, self, Vector2(10,10), joints)
        self.controlBars.update()
        #Coord inputs
        Label(self.root, text="X").place(x=35, y=180)
        self.coords.append(Entry(self.root, width=7))
        self.coords[-1].place(x=20, y = 200)
        Label(self.root, text="Y").place(x=105, y=180)
        self.coords.append(Entry(self.root, width=7))
        self.coords[-1].place(x=90, y=200)
        Label(self.root, text="Z").place(x=185, y=180)
        self.coords.append(Entry(self.root, width=7))
        self.coords[-1].place(x=170, y=200)

        #Go button
        self.widgets.append(Button(self.root, text="Go", command=self.goToCoords))
        self.widgets[-1].place(x=220, y=200)

    def goToClick(self, e):
        print(e.x, e.y)
        horizontal, vertical = self.visualiser.pixelToCoord(e.x, e.y)

        x = sin(self.solver.baseAngle)*horizontal
        y = cos(self.solver.baseAngle)*horizontal
        z = vertical
        self.coords[0].delete(0,END)
        self.coords[0].insert(0, str(round(x,1)))
        self.coords[1].delete(0,END)
        self.coords[1].insert(0, str(round(y,1)))
        self.coords[2].delete(0,END)
        self.coords[2].insert(0, str(round(z,1)))

        self.sim.target = Vector2(horizontal, z)

        angles, reached = self.sim.getJointState()

        self.solver.setJointState(angles)
        newAngles = self.solver.solveForTarget2(Vector3(x, y, z))
        self.setJointState(newAngles)
        self.controlBars.setTargets(newAngles)

    def goToCoords(self):
        try:
            c = self.coords[0].get()
            x = float(c)
        except:
            x = 0
        try:
            y = float(self.coords[1].get())
        except:
            y = 0
        try:
            z = float(self.coords[2].get())
        except:
            z = 0

        self.sim.target = Vector2(Vector2(x, y).Mag(), z)


        angles, reached = self.sim.getJointState()
        self.solver.setJointState(angles)
        newAngles = self.solver.solveForTarget2(Vector3(x, y, z))
        self.setJointState(newAngles)
        self.controlBars.setTargets(newAngles)

    def setJointState(self, angles):

        pub = rospy.Publisher("arm_controller/command", Float64MultiArray, queue_size=10)
        armNames = rospy.get_param("/arm_controller/joints")
        data = rospy.wait_for_message("joint_states", JointState)
        arr = []
        for n in armNames:
            for d in range(len(data.name)):
                if(data.name[d] == n):
                    arr.append(data.position[d])
        
        arr[1] = -radians(angles[0])
        arr[2] = -radians(angles[1])
        arr[3] = -radians(angles[2])
        print(arr)
        packet = Float64MultiArray(data = arr)
        pub.publish(packet)

        self.sim.setJointState(angles)

    def getJointAngles(self):
        armNames = rospy.get_param("/arm_controller/joints")
        data = rospy.wait_for_message("joint_states", JointState)
        arr = []
        for n in armNames:
            for d in range(len(data.name)):
                if(data.name[d] == n):
                    arr.append(-data.position[d])
        arr = arr[1:]
        return arr

    def setEffectorAngle(self, angle):
        print("setting angle: ", angle)
        self.solver.setEffectorAngle(angle)

    def update(self):
        self.root.update()
        angles, reached = self.sim.getJointState()
        angles = self.getJointAngles()
        self.controlBars.setValues(angles)
        self.controlBars.update()
        self.angleControl.update()
        self.visualiser.clear()
        self.solver.draw(self.visualiser)
        self.preview.setJointState(angles)
        self.preview.update()
        self.preview.drawArm(self.visualiser, fill="red")
        self.visualiser.display()




