#!/usr/bin/env python
from Tkinter import *
from VectorClass import Vector2

class Display(object):
    def __init__(self, root, h = 1000, w = 1000, pos=Vector2(0,0)):
        self.root = root
        self.root.minsize(width=w, height=h)

        self.canvas = Canvas(self.root, height= h, width = w, bg = "dark grey")
        self.canvas.place(x=pos.x, y=pos.y)

        self.h = h
        self.w = w
        self.scale = 0.6
        self.screenOrigin = Vector2(w/2, h/2)

        self.drawGround = True

        self.root.bind("<MouseWheel>", self.scrollScale)

    def setScreenOrigin(self, xFrac, yFrac):
        self.screenOrigin = Vector2(self.w*xFrac, self.h*yFrac)

    def ToggleGround(self, value = True):
        self.drawGround = value

    def drawLine(self, x1, y1, x2, y2, fill=None):
        x1, y1 = self.scaleCoord(x1, y1)
        x2, y2 = self.scaleCoord(x2, y2)
        self.canvas.create_line(x1, y1, x2, y2, fill=fill)

    def drawCircle(self, x, y, r, fill = None):
        x, y = self.scaleCoord(x, y)
        #print(x,y)
        self.canvas.create_oval(x-r, y-r, x+r, y+r, fill=fill)

    def drawPolygon(self, vertexes, fill = None):
        p = []
        for v in vertexes:
            x, y = self.scaleCoord(v.x, v.y)
            p.append((x,y))
        self.canvas.create_polygon(p,fill=fill, stipple="gray50")

    def scaleCoord(self, x, y):
        newX = (x * self.scale) + self.screenOrigin.x
        newY = self.screenOrigin.y - (y * self.scale)
        return newX, newY

    def pixelToCoord(self, x, y):
        newX = (x - self.screenOrigin.x)/self.scale
        newY = (self.screenOrigin.y-y)/self.scale
        return newX, newY

    def drawOrigin(self):
        size = 10/self.scale
        self.drawLine(0,-size,0,size)
        self.drawLine(-size,0,size,0)
        self.canvas.create_line(0, self.screenOrigin.y, self.w, self.screenOrigin.y)
        #self.drawLine(0,0, self.w, 0)

    def scrollScale(self, event):
        amount = event.delta/1200
        self.scale += amount
        self.scale = max(0.1, self.scale)
        self.scale = min(10, self.scale)

    def clear(self):
        self.canvas.delete(ALL)

    def display(self):
        self.drawOrigin()
        self.canvas.update()
        self.root.update()
