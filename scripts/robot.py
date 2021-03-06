#!/usr/bin/env python
from math import *
from VectorClass import Vector2, Vector3
import time
from std_msgs.msg import Float64
import rospy

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
    def __init__(self, parent, child, limits=[-150,150]):
        self.parent = parent
        self.child = child
        self.angle = 0
        self.min = radians(limits[0])
        self.max = radians(limits[1])

    def setAngle(self, val):
        if(val < self.min):
            #print("Too low")
            val = self.min
            pass
        if(val > self.max):
            #print("Too high, max = ", self.max, " val = ", val)
            val = self.max
            pass

        self.angle = val

    def setMax(self):
        self.angle = self.max
        print("Setting Max Angle: ", str(self.max))

    def setMin(self):
        self.angle = self.min
        print("Setting Min Angle: ", str(self.min))

class Robot(object):
    def __init__(self):
        self.joints = []
        self.links = [Link(0, None)]
        self.totalLength = 0.0
        self.effectorAngle = 0.0
        self.transferHeight = 200
        self.maxBounds = []
        self.minBounds = []

        self.baseAngle = 0

        self.target = Vector2(0,0)

    def addLink(self, length, limits=[-150,150]):
        self.links.append(Link(length, self.links[-1]))
        self.joints.append(Joint(self.links[-2], self.links[-1], limits=limits))
        self.totalLength += length

    def setJointAngle(self, index, value):
        self.joints[index].setAngle(value)

    def getJointAngle(self, index):
        return self.joints[index].angle

    def getJointStates(self):
        state = [self.baseAngle]
        for j in self.joints:
            state.append(degrees(j.angle))
        return state

    def setJointState(self, angles):
        self.baseAngle = angles[0]
        for a in range(len(angles)-1):
            self.joints[a].setAngle(angles[a+1])
        self.update()

    def setEffectorAngle(self, angle):
        startAngles = self.getJointStates()
        self.effectorAngle = angle
        self.getLimits()
        self.update()

        target = self.links[-1].end
        endAngles = self.solveForTarget2(Vector3(target.x*cos(self.baseAngle), target.x*sin(self.baseAngle), target.y))
        
        path = self.interpolateMovement(startAngles, endAngles)
        
        return path

    def update(self):
        totalAngle = 0
        for j in self.joints:
            l1 = j.parent
            l2 = j.child
            totalAngle += j.angle

            l2.pos.x = l1.end.x
            l2.pos.y = l1.end.y

            l2.end.x = l2.pos.x + (sin(totalAngle) * l2.length)
            l2.end.y = l2.pos.y + (cos(totalAngle) * l2.length)
            l2.angle = atan2((l2.end - l2.pos).y, (l2.end - l2.pos).x)

    def ccd(self, worldTarget):

        target = Vector2(Vector2(worldTarget.x, worldTarget.y).Mag(), worldTarget.z)
        x = target.x
        y = target.y

        originalAngles = []
        for j in self.joints:
            originalAngles.append(j.angle)

        searchDepth = 500
        tolerance = 0.1

        #self.joints[0].setAngle(radians(0))
        #self.joints[1].setAngle(radians(0))
        #self.joints[2].setAngle(radians(0))
        self.update()

        target = Vector2(x,y)
        effectorTarget = Vector2(target.x - self.links[-1].length * cos(self.effectorAngle),
                                 target.y - self.links[-1].length * sin(self.effectorAngle))  # EffectorTarget

        end = self.links[-1].end

        difference = effectorTarget-end
        dist = difference.Mag()
        n = 0

        while dist > tolerance and n < searchDepth:
            n += 1
            print(n)
            for j in range(len(self.joints)-1, -1, -1):
                joint = self.joints[j]
                link = joint.child

                Vt = target - link.pos
                #display.drawLine(link.pos.x, link.pos.y, link.pos.x + Vt.x, link.pos.y + Vt.y, fill="red")

                Ve = end - link.pos
                #display.drawLine(link.pos.x, link.pos.y, link.pos.x + Ve.x, link.pos.y + Ve.y, fill="green")

                theta = acos(Vt.Normalise().Dot(Ve.Normalise()))

                theta = Vt.Angle() - Ve.Angle()

                joint.setAngle(joint.angle - theta)
                self.update()

                self.joints[2].setAngle(-(self.effectorAngle - self.links[-2].angle))
                self.update()

                #display.drawCircle(target.x, target.y, 10)

                #self.draw(display)
                #display.display()
                #display.clear()

                end = self.links[-1].end
                difference = target - end
                dist = difference.Mag()
                if(dist < tolerance):
                    invalid = False
                    for j in self.joints:
                        if(j.angle > j.max or j.angle < j.min):
                            j.setAngle(0)
                            invalid = True
                            print("invalid")
                        self.update()
                    if invalid == False:
                        return

        '''
        while dist > tolerance and n < searchDepth:
            n += 1
            for j in range(len(self.joints)-2, 0, -1):
                joint = self.joints[j]
                link = joint.child

                Vt = effectorTarget-link.pos
                display.drawLine(link.pos.x, link.pos.y, link.pos.x+Vt.x, link.pos.y+Vt.y, fill = "red")

                Ve = end-link.pos
                display.drawLine(link.pos.x, link.pos.y, link.pos.x+Ve.x, link.pos.y+Ve.y, fill = "green")

                time.sleep(5)
                self.draw(display)
                display.drawCircle(effectorTarget.x, effectorTarget.y, 10)
                display.display()
                display.clear()

                theta = acos(Vt.Normalise().Dot(Ve.Normalise()))
                joint.setAngle(joint.angle+theta)
                self.joints[2].setAngle(-self.links[-2].angle)
                self.update()

                end = self.links[2].end
                difference = effectorTarget - end
                dist = difference.Mag()


                print(difference, dist, n)
        '''

    def checkRange(self, target):
        dist = target.Mag()

        totalLength = 0
        for l in self.links:
            totalLength += l.length
        if(dist > totalLength):
            print("Too far away")

    def getLimits(self):
        resolution = 1
        points = []
        self.minBounds = []
        self.maxBounds = []

        a = self.links[2].length
        c = self.links[1].length
        a2 = a*a
        c2 = c*c
        maxLength = a+c
        minLength = sqrt(a2+c2-(2*a*c*cos(pi-self.joints[1].min)))

        effectorVector = Vector2(self.links[-1].length*cos(self.effectorAngle), self.links[-1].length*sin(self.effectorAngle))

        for i in range(181,int(degrees(pi+self.joints[1].min)), -resolution):
            theta = radians(i)
            p = Vector2(a*cos(theta)-c, a*sin(theta)) + effectorVector
            if(p.x >= 0 and p.y >= effectorVector.y and p.y >= 0):
                points.append(p)
                self.minBounds.append(p)

        for i in range(181,-181,-resolution):
            theta = radians(i)
            maxi = Vector2(cos(theta)*maxLength, sin(theta)*maxLength) + effectorVector
            if(maxi.x >= 0 and maxi.y >= effectorVector.y and maxi.y >= 0):
                points.append(maxi)
                self.maxBounds.append(maxi)

            if(theta <= pi+asin((sin(self.joints[1].min)*a)/minLength)):
                mini = Vector2(cos(theta) * minLength, sin(theta) * minLength) + effectorVector

                if(mini.x >= 0 and mini.y >= effectorVector.y and mini.y >= 0):
                    points.append(mini)
                    self.minBounds.append(mini)

        if(len(self.minBounds) == 0):
            self.minBounds.append(Vector2(0,0))
        vec = (self.maxBounds[-1]-self.minBounds[-1])/10
        for i in range(10):
            p = self.minBounds[-1]+vec
            p.y = max(p.y, 0)
            self.minBounds.append(p)


        return self.minBounds, self.maxBounds

    def isOutsideLimits(self, target):
        #Check its falls between limits
        if(target.x > self.minBounds[-1].x or target.x < self.minBounds[0].x):
            if(target.x > self.maxBounds[-1].x or target.x < self.maxBounds[0].x):
                
                print("Beyond X")
                print(target.x)
                return True

        #Find where it falls between min bounds
        for i in range(len(self.minBounds)):
            if self.minBounds[i].x > target.x:
                b1 = self.minBounds[i-1]
                b2 = self.minBounds[i]
                break

        #Interpolate
        frac = (target.x-b1.x)/(b2.x-b1.x)
        minY = b1.y + (b2.y-b1.y)*frac

        #Find where it falls between max bounds
        for i in range(len(self.maxBounds)):
            if self.maxBounds[i].x > target.x:
                b1 = self.maxBounds[i-1]
                b2 = self.maxBounds[i]
                break

        #Interpolate
        frac = (target.x-b1.x)/(b2.x-b1.x)
        maxY = b1.y + (b2.y-b1.y)*frac


        if(target.y < minY or target.y > maxY):
            return True
        else:
            return False

    def solveForEffectorPosition(self, angles):
        self.update()
        return self.links[-1].end

    def solveForTarget2(self, worldTarget):
        target = Vector2(Vector2(worldTarget.x, worldTarget.y).Mag(), worldTarget.z)  # TargetVector
        self.target = target
        print("TARGET: ", target.x, target.y)

        originalAngles = [self.baseAngle]
        for j in self.joints:
            originalAngles.append(degrees(j.angle))


        #Check its within limits
        if(self.isOutsideLimits(target)):
            print("Out of bounds")
            return originalAngles

        self.baseAngle = atan2(worldTarget.y, worldTarget.x)

        effectorTarget = Vector2(target.x - self.links[-1].length * cos(self.effectorAngle),
                                 target.y - self.links[-1].length * sin(self.effectorAngle))  # EffectorTarget

        #self.target = effectorTarget

        a = self.links[2].length
        c = self.links[1].length
        b = effectorTarget.Mag()

        a2 = a*a
        b2 = b*b
        c2 = c*c

        A = acos((b2+c2-a2)/(2*b*c))
        B = acos((a2+c2-b2)/(2*a*c)) - pi
        A = atan2(effectorTarget.x, effectorTarget.y) - A

        self.joints[0].setAngle(A)
        self.joints[1].setAngle(-B)
        self.update()

        self.joints[2].setAngle(-(self.effectorAngle - self.links[-2].angle))
        self.update()
        end = self.links[2].end
        difference = effectorTarget - end
        dist = difference.Mag()

        if (dist > 100):
            self.baseAngle = originalAngles[0]
            for j in range(1,len(originalAngles)):
                self.joints[j].setAngle(originalAngles[j])
            self.update()

        state = [degrees(self.baseAngle)]
        for j in self.joints:
            state.append(degrees(j.angle))

        return state

    def getPathToTarget(self, worldTarget,  steps = 50):

        self.update()

        startPos = self.links[-1].end
        p1 = Vector3(startPos.x*cos(self.baseAngle), startPos.x*sin(self.baseAngle), startPos.y)

        p2 = Vector3(100*cos(self.baseAngle), 100*sin(self.baseAngle), 450)

        targetAngle = atan2(worldTarget.y, worldTarget.x)
        mag = Vector2(p2.x, p2.y).Mag()

        p3 = Vector3(mag*cos(targetAngle), mag*sin(targetAngle), 450)

        p4 = worldTarget

        self.points = []
        
        t1 = degrees(targetAngle) % 360
        t2 = degrees(self.baseAngle) % 360 

        if( abs(t1-t2) > 10):
            points = [p1,p2,p3,p4]

        else:
            points = [p1, p4]

        path = []
        for p in points:
            path.append(self.solveForTarget2(p))
        return path

    def interpolateMovement(self, startAngles, endAngles, steps = 20):
        path = []
        intervals = []

        for a in range(len(startAngles)):
            intervals.append((endAngles[a]-startAngles[a])/steps)

        path.append(startAngles)
        for i in range(0,steps-1):
            position = []
            for a in range(len(startAngles)):
                position.append(startAngles[a]+intervals[a]*i)
            path.append(position)
        #path.append(endAngles)

        return path

    def drawArm(self, dis, fill="black"):
        self.update()
        for l in self.links:
            dis.drawLine(l.pos.x, l.pos.y, l.end.x, l.end.y, fill=fill)
        pub = rospy.Publisher("gripper/angle", Float64, queue_size=10)
        data = Float64(self.links[-1].angle)
        pub.publish(data)


    def draw(self, dis):
        vertexArray = []
        for i in self.minBounds:
            vertexArray.append(i)
        for i in range(len(self.maxBounds)-1, -1, -1):
            vertexArray.append(self.maxBounds[i])
        dis.drawPolygon(vertexArray, fill="#32CD32")

        for l in self.links:
            dis.drawLine(l.pos.x, l.pos.y, l.end.x, l.end.y)
        dis.drawCircle(self.links[-1].end.x, self.links[-1].end.y,10)
        dis.drawCircle(self.target.x, self.target.y, 5, fill="red")