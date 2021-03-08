#!/usr/bin/env python

import display, robot, math, time, VectorClass, simulator, controller
from Tkinter import *
from math import *
import tf2_py
import tf2_ros
import rospy, os, time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import geometry_msgs.msg

import roslib; roslib.load_manifest('urdfdom_py')
from urdf_parser_py.urdf import URDF


class RadialDisplay(object):
    def __init__(self, root, solver, position):
        self.root = root
        self.solver = solver
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
        self.solver.setEffectorAngle(-angle)
        self.input.delete(0,END)
        self.input.insert(0, str(round(degrees(angle),1)))

    def setAngleFromEntry(self):
        self.angle = float(self.input.get())
        self.solver.setEffectorAngle(-radians(self.angle))


class CoordinateControls(object):
    def __init__(self, root, controller, solver, position, joints):

        self.root = root
        self.controller = controller
        self.solver = solver
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

        self.entries = []
        self.axisLabels = []

        #Coord inputs
        Label(self.root, text="X").place(x=position.x+35, y=position.y+180)
        self.entries.append(Entry(self.root, width=7))
        self.entries[-1].place(x=position.x+20, y = position.y+200)
        Label(self.root, text="Y").place(x=position.x+105, y=position.y+180)
        self.entries.append(Entry(self.root, width=7))
        self.entries[-1].place(x=position.x+90, y=position.y+200)
        Label(self.root, text="Z").place(x=position.x+185, y=position.y+180)
        self.entries.append(Entry(self.root, width=7))
        self.entries[-1].place(x=position.x+170, y=position.y+200)

        #Go button
        b = Button(self.root, text="Go", command=self.goToCoords)
        b.place(x=position.x+220, y=position.y+200)

        print(joints)
        for j in joints:
            self.addVisualiser([j[0],j[1][0],j[1][1]])

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

        angles = self.controller.getJointStates()
        self.solver.setJointStates(angles)
        newAngles = self.solver.solveForTarget2(Vector3(x, y, z))
        self.controller.setSetpoints(newAngles)
        self.controlBars.setTargets(newAngles)


    def addVisualiser(self, joint):
        p = self.position + VectorClass.Vector2(0,60)*self.n
        self.n += 1
        print(joint)

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
        self.controller.setSetpoints(self.getTargets())

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

    def setCoords(self, x, y, z):
        self.entries[0].delete(0,END)
        self.entries[0].insert(0, str(round(x,1)))
        self.entries[1].delete(0,END)
        self.entries[1].insert(0, str(round(y,1)))
        self.entries[2].delete(0,END)
        self.entries[2].insert(0, str(round(z,1)))

    def update(self):
        angles = self.controller.getJointStates()
        self.setValues(angles)
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
        self.controller = None

    def goToClick(self, e):
        print(e.x, e.y)
        horizontal, vertical = self.horizontalDisplay.pixelToCoord(e.x, e.y)

        x = sin(self.solver.baseAngle)*horizontal
        y = cos(self.solver.baseAngle)*horizontal
        z = vertical

        angles = self.controller.getJointStates()

        self.solver.setJointState(angles)
        newAngles = self.solver.solveForTarget2(VectorClass.Vector3(x, y, z))
        self.controller.setSetpoints(newAngles)
        self.controlBars.setTargets(newAngles)

    def setup(self):
        links = self.GetLinks()
        print("LINKS:", links)

        self.controller = controller.Controller()
        self.controller.update()
        angles = self.controller.getJointStates()

        self.horizontalDisplay = display.Display(self.root, h=300,w=600, pos=VectorClass.Vector2(0,300))
        self.horizontalDisplay.canvas.bind("<Button-1>", self.goToClick)
        self.horizontalDisplay.setScreenOrigin(0.25, 0.8)

        self.controlBars = CoordinateControls(self.root, self.controller, self.solver, VectorClass.Vector2(10,10), links)
        self.controlBars.setTargets(angles)
        self.controlBars.update()

        self.angleControl = RadialDisplay(self.root, self.solver, VectorClass.Vector2(300,10))

        for l in links:
            self.solver.addLink(l[0], l[1])
            self.preview.addLink(l[0], l[1])
        self.solver.setEffectorAngle(0)
        
    def GetLinks(self):
        jointNames = rospy.get_param("/arm_controller/joints")
        robot = URDF.from_xml_string(rospy.get_param("/robot_description"))

        class Link:
            def __init__(self, name):
                self.name = name
                self.length = None
                self.joint = None


        class Joint:
            def __init__(self, name):
                self.name = name
                self.limits = None
                self.parent = None
                self.child = None

        def getTransform(link1, link2, timeout=5):
            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)
            found = False
            while found == False:
                try:
                    trans = tfBuffer.lookup_transform(link1, link2, rospy.Time(), rospy.Duration(timeout))
                    
                    found = True
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    print("error")
                    return False
            return trans


        def getLink(name):
            for l in links:
                if(l.name == name):
                    return l

        links = []
        linkNames = ['world']
        links.append(Link("world"))
        
        joints = []
        for n in jointNames:
            for j in robot.joints:
                if(j.name == n):

                    joint = Joint(j.name)
                    joint.limits = [j.limit.lower, j.limit.upper]

                    l1 = getLink(j.parent)
                    l2 = Link(j.child)

                    joint.parent = l1
                    joint.child = l2
                    l2.joint = joint

                    joints.append(joint)
                    links.append(l2)

                    linkNames.append(j.child)

        print(links)
        print("Links:")
        for l in range(len(links)-1):
            l1 = links[l]
            l2 = links[l+1]
            trans = getTransform(l1.name, l2.name)
            if(trans):
                print(trans.transform.translation)
                l1.length = trans.transform.translation.x * 1000
        links[-1].length = 65

        links = links[2:]

        arr = []
        for l in links:
            
            joint = l.joint
            lim = [-degrees(joint.limits[1]), -degrees(joint.limits[0])]
            arr.append([l.length, lim])

        return arr

    def update(self):
        self.controller.update()
        self.root.update()
        self.controller.update()

        self.controlBars.update()
        self.angleControl.update()
        self.horizontalDisplay.clear()

        self.preview.setJointState(self.controller.getJointStates())
        self.preview.update()

        self.solver.draw(self.horizontalDisplay)
        self.preview.drawArm(self.horizontalDisplay, fill="red")
        self.horizontalDisplay.display()
        



if __name__ == '__main__':
    rospy.init_node('newController', anonymous=True)
    rospy.logerr("controller starting")
    rate = rospy.Rate(50)
    panel = ControlPanel()
    panel.setup()

    while not rospy.is_shutdown():
        panel.update()
        rate.sleep()