#!/usr/bin/env python

print("HELLLLLLLLLLLLOOOOOOOOOOOOOOOO")

'''
import display, robot, math, time, VectorClass, simulator, controller
from Tkinter import *
import rospy

def getAngle(event):
    global angle, target
    target = [max(1,event.x-500),max(1,500-event.y)]
    target = [event.x-500, 500-event.y]
    x = event.x - 500
    y = 500 - event.y
    angle = math.atan2(x,y)

def checkReached():
    state = s.getJointState()
    print(state[1])
    for r in state[1]:
        if(r == False):
            return False
    return True

#if __name__ == '__main__':
rospy.init_node('newController', anonymous=True)
rate = rospy.Rate(10)

target = [200,100]
angle = 0

root = Tk()
d = display.Display(root)
#r = robot.Robot()
s = simulator.Sim()

#r.addLink(150, limits=[-180,180])
s.addLink(150, limits=[0,180])
#r.addLink(120)
s.addLink(120)
#r.addLink(100)
s.addLink(100)

c = controller.Controller(s)


while not rospy.is_shutdown():
    try:
        s.update(0.01)
        c.update()
        d.clear()

        # for l in c.solver.minBounds:
            #   d.drawCircle(l.x, l.y, 2, fill="green")
            #for l in c.solver.maxBounds:
                #d.drawCircle(l.x, l.y, 2, fill="red")
        s.draw(d)
        d.display()
        rate.sleep()
    except rospy.ROSInterruptException:
        print("Oh no")


r.calcBs(d)
r.setJointAngle(0, math.radians(0))
r.setJointAngle(1, math.radians(45))
r.setJointAngle(2, math.radians(45))

n = 0
jointPositions = [[0,0,0],[45,0,45],[90,45,-45]]

while True:
    d.clear()
    s.update(0.001)

    states = s.getJointState()
    if(checkReached()):
        time.sleep(1)
        s.setJointState(jointPositions[n])
        print("Moving to:", jointPositions[n])
        n+=1

    s.draw(d)
    d.display()


target = VectorClass.Vector3(100,100,100)
d.clear()
#r.moveToTarget3(target[0], 0, target[1], d)
r.update()
r.draw(d)
d.drawCircle(target[0],target[1], 10, fill = "red")
d.display()
r.effectorAngle = math.radians(-10)
#while True:
#    d.display()




while True:
    d.clear()
    try:
        r.moveToTarget3(target[0], 0, target[1], d)
    except:
        print("Error")
    r.update()
    r.draw(d)
    #d.drawLine(0,0,-100,-300)
    d.drawCircle(target[0], target[1], 10)
    d.display()

'''

