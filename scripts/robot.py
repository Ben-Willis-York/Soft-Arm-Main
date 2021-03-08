#!/usr/bin/env python
from math import *
from VectorClass import Vector2, Vector3
import time

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

    def addLink(self, length, limits=[-150,150]):
        print(length, limits)
        self.links.append(Link(length, self.links[-1]))
        self.joints.append(Joint(self.links[-2], self.links[-1], limits=limits))
        self.totalLength += length

    def setJointAngle(self, index, value):
        self.joints[index].setAngle(value)

    def getJointAngle(self, index):
        return self.joints[index].angle

    def setJointState(self, angles):
        self.baseAngle = angles[0]
        for a in range(len(angles)-1):
            self.joints[a].setAngle(angles[a+1])
        self.update()

    def setEffectorAngle(self, angle):
        self.effectorAngle = angle
        self.getLimits()

    def update(self):
        totalAngle = 0
        for j in self.joints:
            l1 = j.parent
            l2 = j.child
            totalAngle += j.angle

            l2.pos.x = l1.end.x
            l2.pos.y = l1.end.y

            l2.end.x = l2.pos.x + (cos(totalAngle) * l2.length)
            l2.end.y = l2.pos.y + (sin(totalAngle) * l2.length)
            l2.angle = atan2((l2.end - l2.pos).y, (l2.end - l2.pos).x)

        '''
        for i in range(1, len(self.links)):
            l1 = self.links[i-1]
            l2 = self.links[i]
            j = self.joints[i-1]
            totalAngle = l1.angle+j.angle

            l2.pos.x = l1.end.x
            l2.pos.y = l1.end.y

            l2.end.x = l2.pos.x+(cos(totalAngle) * l2.length)
            l2.end.y = l2.pos.y+(sin(totalAngle) * l2.length)
            l2.angle = atan2((l2.end-l2.pos).y, (l2.end-l2.pos).x)
            #l2.angle = totalAngle
        '''

    def ccd(self, x, y, display):
        searchDepth = 50
        tolerance = 5

        self.joints[0].setAngle(radians(0))
        self.joints[1].setAngle(radians(0))
        self.joints[2].setAngle(radians(0))
        self.update()

        target = Vector2(x,y)
        effectorTarget = Vector2(target.x - self.links[-1].length, target.y)

        end = self.links[-1].end

        difference = effectorTarget-end
        dist = difference.Mag()
        n = 0

        while dist > tolerance and n < searchDepth:
            n += 1
            n += 1
            for j in range(len(self.joints)-1, -1, -1):
                joint = self.joints[j]
                link = joint.child

                Vt = target - link.pos
                #display.drawLine(link.pos.x, link.pos.y, link.pos.x + Vt.x, link.pos.y + Vt.y, fill="red")

                Ve = end - link.pos
                #display.drawLine(link.pos.x, link.pos.y, link.pos.x + Ve.x, link.pos.y + Ve.y, fill="green")

                theta = acos(Vt.Normalise().Dot(Ve.Normalise()))
                joint.setAngle(joint.angle + theta)
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

        print("Between:", minY, maxY)

        if(target.y < minY or target.y > maxY):
            return True
        else:
            return False

    def solveForEffectorPosition(self, angles):
        self.update()
        return self.links[-1].end

    def solveForTarget2(self, worldTarget):
        target = Vector2(Vector2(worldTarget.x, worldTarget.y).Mag(), worldTarget.z)  # TargetVector
        

        originalAngles = [self.baseAngle]
        for j in self.joints:
            originalAngles.append(degrees(j.angle))


        #Check its within limits
        if(self.isOutsideLimits(target)):
            print("Original Angles = ", originalAngles)
            print("Out of bounds")
            return originalAngles

        self.baseAngle = atan2(worldTarget.y, worldTarget.x)
        effectorTarget = Vector2(target.x - self.links[-1].length * cos(self.effectorAngle),
                                 target.y - self.links[-1].length * sin(self.effectorAngle))  # EffectorTarget

        a = self.links[2].length
        c = self.links[1].length
        b = effectorTarget.Mag()

        a2 = a*a
        b2 = b*b
        c2 = c*c

        A = acos((b2+c2-a2)/(2*b*c))
        B = pi - acos((a2+c2-b2)/(2*a*c))
        A += atan2(effectorTarget.y, effectorTarget.x)

        self.joints[0].setAngle(A)
        self.joints[1].setAngle(-B)
        self.update()

        self.joints[2].setAngle(self.effectorAngle - self.links[-2].angle)
        self.update()
        end = self.links[2].end
        difference = effectorTarget - end
        dist = difference.Mag()

        if (dist > 100):
            print("out of range")
            self.baseAngle = originalAngles[0]
            for j in range(1,len(originalAngles)):
                self.joints[j].setAngle(originalAngles[j])
            self.update()

        state = [degrees(self.baseAngle)]
        for j in self.joints:
            state.append(degrees(j.angle))
        return state

    def getPathToTarget(self, worldTarget, steps = 70):
        startAngles = [degrees(self.baseAngle)]
        
        for j in self.joints:
            startAngles.append(degrees(j.angle))

        print(startAngles)

        endAngles = self.solveForTarget2(worldTarget)

        print(endAngles)

        intervals = []
        path = []
        path.append(startAngles)
        
        for a in range(len(startAngles)):
            intervals.append((endAngles[a]-startAngles[a])/steps) 

        for i in range(1,steps):
            position = []
            for a in range(len(startAngles)):
                position.append(startAngles[a]+intervals[a]*i)
            path.append(position)

        for i in range(len(path)):
            self.baseAngle = path[i][0]
            self.joints[0].setAngle(radians(path[i][1]))
            self.joints[1].setAngle(radians(path[i][2]))
            self.update()
            path[i][-1] = degrees(self.effectorAngle - self.links[-2].angle)

        path.append(endAngles)

        return path


    '''
    def solveForTarget(self, worldTarget):
        #baseAngle = tan(worldTarget.x / worldTarget.y)
        target = Vector2(Vector2(worldTarget.x, worldTarget.y).Mag(), worldTarget.z)  # TargetVector
        originalAngles = []
        for j in self.joints:
            originalAngles.append(j.angle)

        searchDepth = 500
        tolerance = 1

        effectorTarget = Vector2(target.x - self.links[-1].length * cos(self.effectorAngle),
                                 target.y - self.links[-1].length * sin(self.effectorAngle))  # EffectorTarget

        end = self.links[2].end
        difference = effectorTarget - end
        dist = difference.Mag()

        if (dist < tolerance):
            return

        # Set initial pose to converge from
        thetaA = radians(90)
        thetaB = radians(0)

        self.joints[0].setAngle(thetaA)
        self.joints[1].setAngle(thetaB)
        self.update()

        #thetaA = self.joints[1].angle
        #thetaB = self.joints[2].angle

        n = 0
        while dist > tolerance and n < searchDepth:
            n += 1

            scaler = dist/1000
            if (difference.x > 0):
                thetaA -= 1 * scaler
                # thetaB += 0.01
            if (difference.x < 0):
                thetaA += 1 * scaler
                # thetaB -= 0.01
            if (difference.y > 0):
                thetaB += 1 * scaler
            if (difference.y < 0):
                thetaB -= 1 * scaler

            self.joints[0].setAngle(thetaA)
            self.joints[1].setAngle(thetaB)
            self.update()
            end = self.links[2].end
            difference = effectorTarget - end
            dist = difference.Mag()



        self.joints[2].setAngle(self.effectorAngle - self.links[-2].angle)
        #self.joints[2].setAngle(pi-45)
        self.update()

        if (dist > tolerance):
            print("out of range")
            for j in range(len(originalAngles)):
                self.joints[j].setAngle(originalAngles[j])
            self.update()

        state = []
        for j in self.joints:
            state.append(degrees(j.angle))
        return state

    def moveToTarget3(self, worldTarget, display):

        baseAngle = tan(worldTarget.x/worldTarget.y)
        target = Vector2(Vector2(worldTarget.x, worldTarget.y).Mag(), worldTarget.z) #TargetVector

        originalAngles = []
        for j in self.joints:
            originalAngles.append(j.angle)

        #Search Characteristics
        searchDepth = 500
        tolerance = 5


        effectorTarget = Vector2(target.x-self.links[-1].length*cos(self.effectorAngle), target.y-self.links[-1].length*sin(self.effectorAngle)) #EffectorTarget


        end = self.links[2].end
        difference = effectorTarget - end
        dist = difference.Mag()

        if(dist < tolerance):
            return

        #Set initial pose to converge from
        thetaA = radians(90)
        thetaB = radians(0)

        self.joints[0].setAngle(thetaA)
        self.joints[1].setAngle(thetaB)
        self.update()

        n = 0
        while dist > tolerance and n < searchDepth:
            n += 1

            if(difference.x > 2):
                thetaA -= 0.01
                #thetaB += 0.01
            if(difference.x < -2):
                thetaA += 0.01
                #thetaB -= 0.01
            if(difference.y > 2):
                thetaB += 0.01
            if(difference.y < -2):
                thetaB -= 0.01

            self.joints[0].setAngle(thetaA)
            self.joints[1].setAngle(thetaB)
            self.update()
            end = self.links[2].end
            difference = effectorTarget - end
            dist = difference.Mag()

            self.joints[2].setAngle(self.effectorAngle+self.links[-2].angle)
            self.update()

            #time.sleep(0.01)
            
            #display.clear()
            #self.draw(display)
            #display.drawCircle(effectorTarget.x, effectorTarget.y, 10)
            #display.display()

            #print(difference, dist)
            
        if(dist > tolerance):
            print("out of range")
            for j in range(len(originalAngles)):
                self.joints[j].setAngle(originalAngles[j])
            self.update()

        print(n)

    def moveToTarget2(self, x, y):
        target = Vector2(x, y)
        end = Vector2(self.links[-1].end[0], self.links[-1].end[1])


        for j in range(len(self.joints)):
            joint = self.joints[j]
            link = joint.child
            pos = Vector2(link.pos[0], link.pos[1])

            a = link.length
            ai = Vector2(link.end[0]-link.pos[0], link.end[1]-link.pos[1])

            b = link.minB

            ci = target-pos
            c = (target-pos).Mag()

            a2 = a*a
            b2 = b*b
            c2 = c*c

            if(c >= a+b):
                ai = ci
                #newEnd = Vector2(link.pos[0], link.pos[1]) + ai
                angle = atan2(ai.y, ai.x)
                joint.setAngle(angle-joint.parent.angle)

            elif( c < abs(a-b)):
                ai = ci*-1
                #newEnd = Vector2(link.pos[0], link.pos[1]) + ai
                angle = atan2(ai.y, ai.x)
                joint.setAngle(angle-joint.parent.angle)
            else:
                print("else")
                #delta = acos((-(b2-a2-c2))/(2*a*c))

                x = -(b2-a2-c2)
                h = 2*a*c
                y = sqrt(h*h - x*x)
                #delta = atan2(y, x)

                delta = atan2(2 * a * c, -(b2 - a2 - c2))

                theta = acos(ai.Normalise().Dot(ci.Normalise())) - delta

                ain = ai.Normalise()
                cin = ci.Normalise()

                #theta = atan2(ain.y-cin.y, ain.x-cin.x)
                #theta = atan2(cin.y, cin.x) - atan2(ain.y, ain.x) - delta

                dot = ain.Dot(cin)
                det = ain.Det(cin)
                theta = atan2(det, dot) - delta

                joint.setAngle(joint.angle + theta)


            print(degrees(joint.angle))
            self.update()

    def moveToTarget(self, x, y):
        target = [x,y]
        #print(target)

        end = self.links[-1].end

        for j in range(len(self.joints)):
            joint = self.joints[j]
            link = joint.child
            pos = link.pos

            Vt = Vector2(target[0]-pos[0], target[1]-pos[1])
            Ve = Vector2(end[0]-link.pos[0], end[1]-link.pos[1])
            print("Vt:", Vt, "Ve:", Ve)

            Vtn = Vt.Normalise()
            Ven = Ve.Normalise()
            dot = Vtn.Dot(Ven)
            det = Vtn.Det(Ven)
            theta = atan2(det, dot)


            a = link.length
            b = 0
            for i in range(j+1, len(self.joints)):
                b += self.joints[i].child.length

            c = Vt.Mag()

            print(degrees(theta), a, b, c)

            omegaI = radians(180)

            a2 = a * a
            b2 = b * b
            c2 = c * c

            if(c >= (a+b)):
                print("Adding")
                joint.angle += theta
                self.update()
            elif(c < abs(a-b)):
                print("Subbing")
                joint.angle += -theta
                self.update()

            
            # elif(a2+b2-c2 > 0 and a != 0 and b != 0):

            #     print("beyond")
            #     phiB = acos((a2 + c2 - b2) / (2 * a * c))
            #     phiC = acos((a2 + b2 - c2) / (2 * a * b))
            #     betaI = theta - phiB
            #     betaT = pi - phiC

            #     if(betaT > omegaI):
            #         phiB = phiB-radians(10)
            #     else:
            #         phiB = phiB+radians(10)

            #     betaI = theta + phiB
            #     joint.angle += betaI
            # self.update()    
    '''

    def drawArm(self, dis, fill="black"):
        for l in self.links:
            dis.drawLine(l.pos.x, l.pos.y, l.end.x, l.end.y, fill=fill)

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