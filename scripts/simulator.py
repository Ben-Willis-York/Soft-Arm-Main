#!/usr/bin/env python


from math import *
from VectorClass import Vector2, Vector3
import time
from Tkinter import *

class Link(object):
    def __init__(self, length, parent):
        self.length = length
        self.parent = parent

        self.angle = 0
        self.pos = Vector2(0,0)
        self.end = Vector2(0, length)

        self.B = 0
        self.minB = 0

class Joint(object):
    ID = 1
    def __init__(self, parent, child, limits=[-150,150]):
        self.name = "J"+str(Joint.ID)
        Joint.ID += 1
        self.parent = parent
        self.child = child
        self.angle = 0
        self.targetAngle = 0
        self.min = radians(limits[0])
        self.max = radians(limits[1])
        self.speed = 1
        self.targetReached = False

    def setAngle(self, val):
        if(val < self.min):
            val = self.min
        if(val > self.max):
            val = self.max
        self.targetAngle = val

    def update(self, delta):
        if(abs(self.angle - self.targetAngle) < 0.05):
            self.targetReached = True
        if(self.angle > self.targetAngle):
            self.angle -= self.speed * delta
            self.targetReached = False
        if(self.angle < self.targetAngle):
            self.angle += self.speed * delta
            self.targetReached = False

class Sim(object):
    def __init__(self):
        self.joints = []
        self.links = [Link(0,None)]

        self.target = Vector2(0,0)

    def getJoints(self):
        joints = []
        for j in self.joints:
            joints.append([j.name, degrees(j.min), degrees(j.max)])
        return joints

    def addLink(self, length, limits=[-150,150]):
        print(limits)
        self.links.append(Link(length, self.links[-1]))
        self.joints.append(Joint(self.links[-2], self.links[-1], limits=limits))
        self.update(0.0)

    def setJointState(self, angles):
        for a in range(len(angles)):
            self.joints[a].setAngle(radians(angles[a]))

    def getJointState(self):

        angles = []
        reached = []
        for j in self.joints:
            angles.append(j.angle)
            reached.append(j.targetReached)
        return angles, reached

    def getJointDistance(self, name1, name2):
        for j in self.joints:
            if(j.name == name1):
                j1 = j
            if(j.name == name2):
                j2 = j

        p1 = j1.child.pos
        p2 = j2.child.pos

        return (p2-p1).Mag()

    def getJointLength(self, name):
        for j in self.joints:
            if(name == j.name):
                return (j.child.end-j.child.pos).Mag()

    def update(self, delta):
        for j in self.joints:
            j.update(delta)

        for i in range(1, len(self.links)):
            l1 = self.links[i - 1]
            l2 = self.links[i]
            j = self.joints[i - 1]
            totalAngle = l1.angle + j.angle

            l2.pos.x = l1.end.x
            l2.pos.y = l1.end.y

            l2.end.x = l2.pos.x + (cos(totalAngle) * l2.length)
            l2.end.y = l2.pos.y + (sin(totalAngle) * l2.length)
            l2.angle = totalAngle

    def draw(self, dis):
        for l in self.links:
            dis.drawLine(l.pos.x, l.pos.y, l.end.x, l.end.y)
        #dis.drawCircle(self.links[-1].end.x, self.links[-1].end.y, 10)
        dis.drawCircle(self.target.x, self.target.y, 10, fill="red")