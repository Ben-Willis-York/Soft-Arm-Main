import display, robot, math, time, VectorClass, simulator, controller
from VectorClass import *
from Tkinter import *
from math import *
import tf2_py
import tf2_ros
import roslib
import rospy, os, time, math
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion

import colorsys

import roslib; roslib.load_manifest('urdfdom_py')
from urdf_parser_py.urdf import URDF

JOINT_RESOLUTION = 13
FINGER_LENGTH = 0.15*1000

class Joint(object):
    def __init__(self, name, parent, child, limits=[-150,150]):
        self.name = name
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

class StrainSensor(object):
    def __init__(self, startJoint, endJoint, offset, jointLinks, tfBuffer):
        self.startJoint = startJoint
        self.startCenter = Vector2(0,0)
        self.startInside = Vector2(0,0)
        self.startOutside = Vector2(0,0)


        self.endJoint = endJoint
        self.endCenter = Vector2(0,0)
        self.startInside = Vector2(0,0)
        self.endInside = Vector2(0,0)

        self.offset = offset

        self.insidePositions = []
        self.insideStrain = 1
        self.insideLength = 1

        self.outsidePositions = []
        self.outsideStrain = 1
        self.outsideLength = 1

        self.centerLength = 1

        self.joints = []
        self.tfBuffer = tfBuffer



        current = self.startJoint
        next = jointLinks[self.startJoint].next
        while current != self.endJoint:
            if(next == None):
                print("Sensor ends not connected")
                return
            else:
                self.joints.append(jointLinks[current])
                current = next
                next = jointLinks[next].next
        self.joints.append(jointLinks[self.endJoint])
            
    def getPositions(self):
        startTransform = getTransform(self.tfBuffer, "palm", self.startJoint).transform
        self.startCenter = Vector2(startTransform.y*1000, startTransform.z*1000)

        dir = Vector2(0,0)
        if(jointLinks[startJoint].next):
            t1 = getTransform(self.tfBuffer, "palm", jointLinks[jointLinks[startJoint].next].link).transform
            p1 = Vector2(t1.translation.y*1000, t1.translation.z*1000)
            dir += p1-self.pos


        endTransform = getTransform(self.tfBuffer, "palm", self.endJoint).transform
        self.endCenter = Vector2(endTransform.y*1000, endTransform.z*1000)

    def getStrains(self, jointLinks):
        center = 0
        inside = 0
        outside = 0

        self.outsidePositions = []
        self.insidePositions = []

        for i in range(len(self.joints)-1):
            j1 = self.joints[i]
            j2 = self.joints[i+1]

            outer1 = j1.pos + j1.perp*self.offset
            inner1 = j1.pos - j1.perp*self.offset

            outer2 = j2.pos + j2.perp*self.offset
            inner2 = j2.pos - j2.perp*self.offset

            #dis.drawCircle(outer1.x, outer1.y, 2, "green")
            #dis.drawCircle(inner1.x, inner1.y, 2, "red")

            center += (j2.pos - j1.pos).Mag()
            inside += (inner2 - inner1).Mag()
            outside += (outer2 - outer1).Mag()

            self.outsidePositions.append(outer1)
            self.insidePositions.append(inner1)

        self.outsidePositions.append(outer2)
        self.insidePositions.append(inner2)
        
        self.centerLength = center
        self.insideLength = inside
        self.outsideLength = outside
        
        self.insideStrain = inside/center
        self.outsideStrain = outside/center

    def update(self, jointLinks):
        self.getStrains(jointLinks)

    def draw(self, dis, options):
        if(options["showSides"].get()):
            for p in self.outsidePositions:
                dis.drawCircle(p.x, p.y, 2, fill = "red")
            for p in self.insidePositions:
                dis.drawCircle(p.x, p.y, 2, fill = "green")


class JointImaginary(object):
    def __init__(self, name):
        self.name = name
        self.pos = Vector2(0,0)
        self.dir = Vector2(0,0)
        self.perp = Vector2(0,0)

        self.force = 0

        self.next = None
        self.previous = None

    def draw(self, dis, jointLinks, options, fill="green"):
        if(self.next != None):
            dis.drawLine(self.pos.x, self.pos.y, 
                    jointLinks[self.next].pos.x, jointLinks[self.next].pos.y, fill=fill)
        if options["showNormals"].get():
            forceVec = self.perp  * 20
            dis.drawLine(self.pos.x, self.pos.y, self.pos.x+forceVec.x, self.pos.y+forceVec.y)
            
        dis.drawCircle(self.pos.x, self.pos.y, 1.5, fill=fill)


class JointLink(object):
    def __init__(self, name):
        self.name = name
        self.previous = None
        self.next = None
        self.link = None
        self.pos = Vector2(0,0)
        self.dir = Vector2(0,0)
        self.perp = Vector2(0,0)
        self.inside = Vector2(0,0)
        self.outside = Vector2(0,0)
        self.insideStrain = 0
        self.outsideStrain = 0
        self.force = 0

        self.predictedPos = Vector2(0,0)
        self.predictedDir = Vector2(0,0)
        self.predictedPerp = Vector2(0,0)
        
    def update(self, jointLinks, tfBuffer):
        #print(self.name)
        #print(jointLinks.keys())
        #print 
        transform = getTransform(tfBuffer, "palm", self.link).transform
        self.pos = Vector2(transform.translation.y*1000, transform.translation.z*1000)

        #if(self.previous == None):
        #    dir = Vector2(0,1)

        #else:
        dir = Vector2(0,0)
        if(self.next):
            t1 = getTransform(tfBuffer,"palm", jointLinks[self.next].link).transform
            p1 = Vector2(t1.translation.y*1000, t1.translation.z*1000)
            dir += p1-self.pos

        if(self.previous):
            t1 = getTransform(tfBuffer,"palm", jointLinks[self.previous].link).transform
            p1 = Vector2(t1.translation.y*1000, t1.translation.z*1000)
            
            dir += self.pos-p1

        self.dir = dir.Normalise()
        rot = transform.rotation
        eulers = quaternionToEuler(rot.x, rot.y, rot.z, rot.w)

        self.perp = Vector2(-dir.y*cos(eulers[2]), dir.x*cos(eulers[2])).Normalise() * -1
        self.inside = self.pos + self.perp*10
        self.outside = self.pos - self.perp*10

    def draw(self, dis, jointLinks, fill="red"):
        if(self.next != None):
            dis.drawLine(self.pos.x, self.pos.y, 
                        jointLinks[self.next].pos.x, jointLinks[self.next].pos.y, fill=fill)
        dis.drawCircle(self.pos.x, self.pos.y, 1, fill=fill)




def getJoint(name):
    for j in joints:
        if(j.name == name):
            return j
    return False


class Finger(object):
    def __init__(self, base, jointLinks, tfBuffer):
        self.jointLinks = jointLinks
        self.base = base
        self.baseLongitudeRotation = 0
        self.jointNames = []
        self.realJoints = {}
        self.predictedJoints = {}
        self.expectedJoints = {}
        self.colors = {}
        self.forces = []
        self.sensors = []
        self.K = 0.4
        self.sign = 1

        self.getJoints(tfBuffer)

    def addSensor(self, start, end, offset,tfBuffer):
        if start in self.jointNames and end in self.jointNames:
            self.sensors.append(StrainSensor(start, end, offset, self.jointLinks,tfBuffer))
        else:
            #print("Not available")
            #print(self.jointNames)
            pass

    def getJoints(self,tfBuffer):
        current = self.base
        while current != None:
            self.realJoints[current] = self.jointLinks[current]
            self.jointNames.append(current)
            self.forces.append(0)
            current = self.jointLinks[current].next
        
        for j in self.realJoints.values():
            pred = JointImaginary(j.name)
            pred.next = j.next
            pred.previous = j.previous
            self.predictedJoints[pred.name] = pred

            exp = JointImaginary(j.name)
            exp.next = j.next
            exp.previous = j.previous
            self.expectedJoints[exp.name] = exp

            self.colors[j.name] = [0,0,0]

        t = getTransform(tfBuffer, "palm", self.jointLinks[self.base].link).transform
        eulers = quaternionToEuler(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        self.sign = round(cos(eulers[2]),0)

    def getTipErrors(self):
        joint = self.jointNames[-1]
        error = (self.predictedJoints[joint].pos - self.realJoints[joint].pos).Mag()
        return error
   
    def getForceErrors(self):
        total = 0
        for joint in self.jointNames[1:-1]:
            error = abs(self.predictedJoints[joint].force - self.realJoints[joint].force)
            total += error
        return total/len(self.jointNames)

    def getPoseErrors(self):
        total = 0

        for joint in self.jointNames:
            error = abs((self.predictedJoints[joint].pos - self.realJoints[joint].pos).Mag())
            total += error

        mae = total/len(self.jointNames)

        return mae

    def getBendAngle(self):
        v1 = self.predictedJoints[self.jointNames[0]].dir
        v2 = self.predictedJoints[self.jointNames[-1]].dir
        angle = math.acos(v1.Dot(v2)/(v1.Mag()*v2.Mag()))

        return math.degrees(angle)

    def getRadiusFromLengths(self, length1, length2): 
        if(length1 > length2):
            l1 = length1
            l2 = length2
            sign = -1
        else:
            l1 = length2
            l2 = length1
            sign = 1
        offset = 10

        R = ((l2*offset)/(l1-l2) + offset) * sign
        theta = l1/R
        return R, theta

    def getPrediction(self, dis, options):

        centers = []

        path = []
        normalsPath=[]

        for s in range(len(self.sensors)):
            sensor = self.sensors[s]
            if(s == 0):
                baseJoint = self.predictedJoints[self.sensors[s].startJoint]
                outsideLength, insideLength, centerLength = sensor.outsideLength, sensor.insideLength, sensor.centerLength
            else:
                prev = self.sensors[s-1]
                baseJoint = self.predictedJoints[prev.endJoint]
                outsideLength = sensor.outsideLength - prev.outsideLength
                insideLength = sensor.insideLength - prev.insideLength
                centerLength = sensor.centerLength - prev.centerLength

            endJoint = self.predictedJoints[sensor.endJoint]


            if(outsideLength < insideLength):
                radius, theta = self.getRadiusFromLengths(outsideLength, centerLength)
            else:
                radius, theta = self.getRadiusFromLengths(centerLength,insideLength)

            center = baseJoint.pos + baseJoint.perp * radius
            centers.append(center)

            theta *= self.sign

            globalTheta = theta - baseJoint.perp.Angle()
            end = center + Vector2(radius*cos(pi-globalTheta), radius*sin(pi-globalTheta))

            startTheta = baseJoint.perp.Angle()
            endTheta = theta - startTheta


            res = 50
            interval = theta/res

            for i in range(res):
                globalTheta = pi - interval*i + startTheta
                path.append(center + Vector2(radius*cos(globalTheta), radius*sin(globalTheta)))
                normalsPath.append((center-path[-1]).Normalise() * (radius/abs(radius)))
            path.append(end)
            normalsPath.append(((center-path[-1]).Normalise() * (radius/abs(radius))).Normalise())
            
            endJoint.pos = end
            #endJoint.perp = (center-end).Normalise() * (radius/abs(radius))

            #if options["showNormals"].get():
            #    p2 = endJoint.pos + endJoint.perp*20
            #    dis.drawLine(endJoint.pos.x, endJoint.pos.y, p2.x, p2.y, fill = "red")
            #print("Options:" , options)
            if options["showSensors"].get():
                dis.drawCircle(end.x, end.y, 4, fill="yellow")
            if options["showCenters"].get():
                dis.drawCircle(center.x, center.y, 3, fill="blue")
            if options["showCircles"].get():
                dis.drawCircle(center.x, center.y, radius)
        
        for i in range(res, res+res):
            globalTheta = pi - interval*i + startTheta
            path.append(center + Vector2(radius*cos(globalTheta), radius*sin(globalTheta)))
            normalsPath.append(((center-path[-1]).Normalise() * (radius/abs(radius))).Normalise())
  
        segLength = FINGER_LENGTH/(JOINT_RESOLUTION-1)

        for j in range(len(self.jointNames)):
            targetDist = segLength*j
            dist = 0
            n = 0
            while abs(targetDist-dist) > 1 and targetDist > dist:
                dist += (path[n+1] - path[n]).Mag()
                n+=1
            self.predictedJoints[self.jointNames[j]].pos = path[n]
            self.predictedJoints[self.jointNames[j]].perp = normalsPath[n].Normalise()
        for j in self.predictedJoints.values():
            if j.next != None:
                j.dir = (self.predictedJoints[j.next].pos - j.pos).Normalise()
                #j.perp = j.dir.Perpendicular()
         
        '''END CASE '''
        targetDist = segLength*len(self.jointNames)
        dist = 0
        n = 0
        while abs(targetDist-dist) > 1 and targetDist > dist:
            dist += (path[n+1] - path[n]).Mag()
            n+=1
        self.predictedJoints[self.jointNames[-1]].dir = (path[n+1] - self.predictedJoints[self.jointNames[-1]].pos).Normalise()
        #self.predictedJoints[self.jointNames[-1]].perp = normalsPath[n] * -1

        for p in range(len(path)):
            p1 = path[p]
            p2 = path[p]+(normalsPath[p]*10)
            #dis.drawLine(p1.x, p1.y, p2.x, p2.y)
        
    def getExpected(self, dis, efforts):
        self.K = rospy.get_param("/Design/kConstant")
        segLength = FINGER_LENGTH/(JOINT_RESOLUTION-1)

        #effort = efforts[0]

        next=self.expectedJoints[self.base].next
        while next != None:
            effort = efforts[next]
            prev = self.expectedJoints[self.expectedJoints[next].previous]
            expected = self.expectedJoints[next]

            expectedAngle =  prev.dir.Angle() + (effort/self.K) * self.sign
            
            expected.pos = prev.pos + prev.dir * segLength
            expected.dir = Vector2(cos(expectedAngle), sin(expectedAngle)).Normalise()
            expected.perp = Vector2(-expected.dir.y, expected.dir.x).Normalise() * -1
            next = expected.next

    def update(self,tfBuffer, options):
        self.predictedJoints[self.base].pos = self.realJoints[self.base].pos
        self.predictedJoints[self.base].dir = self.realJoints[self.base].dir
        self.predictedJoints[self.base].perp = self.realJoints[self.base].perp

        self.expectedJoints[self.base].pos = self.realJoints[self.base].pos
        self.expectedJoints[self.base].perp = self.realJoints[self.base].perp
        self.expectedJoints[self.base].dir = self.realJoints[self.base].dir

        for j in self.realJoints.values():
            j.update(self.realJoints, tfBuffer)
        for s in self.sensors:
            s.update(self.realJoints)

    def drawForces(self, dis, options):
        maxSeen = 0
        maxForce = 0.08

        expectedAngles = []
        predictedAngles = []
        relativeAngles = []

        relativeAnglesArr =[]

        totalForceVec = Vector2(0,0)

        for j in self.jointNames:
            expected = self.expectedJoints[j]
            predicted = self.predictedJoints[j]
            
            if(expected.previous):
                expectedRelativeAngle = expected.dir.AngleBetween(self.expectedJoints[expected.previous].dir)
            else:
                expectedRelativeAngle = expected.dir.Angle()
            if(predicted.previous):
                predictedRelativeAngle = predicted.dir.AngleBetween(self.predictedJoints[predicted.previous].dir)
            else:
                predictedRelativeAngle = predicted.dir.Angle()

            relativeAngle = expectedRelativeAngle - predictedRelativeAngle
            #relativeAngle = abs(relativeAngle)
            while relativeAngle > pi:
                relativeAngle -= 2*pi
            while relativeAngle < -pi:
                relativeAngle += 2*pi

            if(relativeAngle >= 0):
                sign = -1
            else:
                sign = 1
       
            relativeAngle = abs(relativeAngle)
            force = self.K * relativeAngle
            col = self.forceToColour(force)
            self.colors[j] = [float(col[0]) ,float(col[1]) ,float(col[2])]

            col = "#%02x%02x%02x" % (col[0] ,col[1] ,col[2])
            self.predictedJoints[j].force = force * self.sign * sign
            self.predictedJoints[j].draw(dis, self.predictedJoints, options, fill=col)  
            totalForceVec += self.predictedJoints[j].perp * force * self.sign * sign

        p1 = self.realJoints[self.base].pos
        p2 = self.realJoints[self.base].pos + totalForceVec * 50
        dis.drawLine(p1.x, p1.y, p2.x, p2.y)
        #print(maxSeen)
        #print(expectedAngles)
        #print(predictedAngles)
        #print(relativeAngles)

    def forceToColour(self, force):
        maxForce = 0.08
        force = abs(force)

        hue = mapper(force, 0, maxForce, 0.3, 1)
        col = colorsys.hsv_to_rgb(1-hue, 1, 1)

        col = (col[0]*255, col[1]*255, col[2]*255)
        return col

    def draw(self, dis, efforts, options):
        self.getPrediction(dis, options)
        self.getExpected(dis, efforts)
        for s in self.sensors:
            s.draw(dis, options)
        if options["showReal"].get():
            for j in self.realJoints.values():
                col = "#%02x%02x%02x" % self.forceToColour(j.force)
                j.draw(dis, self.realJoints, fill=col)
        if options["showExpected"].get():
            for j in self.expectedJoints.values():
                j.draw(dis, self.expectedJoints, options, fill="blue")
        if options["showPredicted"].get():
            for j in self.predictedJoints.values():
                j.draw(dis, self.predictedJoints, options, fill="red")
                
        if options["showForces"].get():
            self.drawForces(dis, options)

class Gripper(object):
    def __init__(self):
        self.jointNames = rospy.get_param("/hand_controller/joints")
        self.robot = URDF.from_xml_string(rospy.get_param("/robot_description"))
        self.link_map = self.robot.link_map
        self.joint_map = self.robot.joint_map
        self.jointLinks = {}
        self.fingerBases = []
        self.sensors = []
        self.fingers  = []

        self.effortInputs = {}
        self.appliedForce = [0,0,0]
        self.jointStates = {}
        self.springStates = {}

        print(self.jointNames)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.Subscriber("/hand_controller/command", Float64MultiArray, self.storeEffortInputs)
        rospy.Subscriber("/link_0_force_fetcher/link_0_forces", Float32MultiArray, self.storeAppliedForce)
        rospy.Subscriber("/joint_states", JointState, self.storeJointStates)
        rospy.Subscriber("/Design/spring_states", Float32MultiArray, self.storeSpringStates)



    def storeSpringStates(self, x):
        for j in range(len(self.jointNames)):
            self.springStates[self.jointNames[j]] = x.data[j]

    def storeJointStates(self, x):
        for i in range(len(x.name)):
            self.jointStates[x.name[i]] = x.effort[i]

    def storeAppliedForce(self, x):
        self.appliedForce= x.data

    def storeEffortInputs(self, x):
   
        for j in range(len(self.jointNames)):
            self.effortInputs[self.jointNames[j]] = x.data[j]

    def getTipDistance(self):
        p1 = self.fingers[0].predictedJoints[self.fingers[0].jointNames[-1]].pos
        p2 = self.fingers[1].predictedJoints[self.fingers[1].jointNames[-1]].pos
        return (p1 - p2).Mag()

    def getBendAngle(self):
        angle = self.fingers[0].getBendAngle()
        return angle

    def setup(self):
        #Add joints with corrosponding links
        for j in self.jointNames:
            self.jointLinks[j] = JointLink(j)
            self.jointLinks[j].link = self.robot.joint_map[j].child

        #Find each joints previous and next neighbour to complete chain
        for j in self.jointLinks.keys():
            link = self.jointLinks[j].link
            for i in self.jointLinks.keys():
                if(self.joint_map[i].child == self.joint_map[j].parent):
                    self.jointLinks[j].previous = i
                if(self.joint_map[i].parent == link):
                    self.jointLinks[j].next = i
            if(self.joint_map[j].parent == "palm"):
                self.fingerBases.append(j)
                self.jointLinks[j].predictedPerp = self.jointLinks[j].perp
                self.jointLinks[j].predictedPos = self.jointLinks[j].pos

        for b in self.fingerBases[1:2]:
            self.fingers.append(Finger(b, self.jointLinks, self.tfBuffer))

        prefixes = ["LT", "LB", "RT", "RB"]
        prefixes = ["LT", "RT", "RB", "LB"]
        for p in prefixes:
            for f in self.fingers:
                
                #f.addSensor("palm_%sseg1_joint" % (p), "%sseg5_%sseg6_joint" % (p,p) , 10, self.tfBuffer)
                
                
                f.addSensor("palm_%sseg1_joint" % (p), "%sseg3_%sseg4_joint" % (p,p),  10, self.tfBuffer)
                f.addSensor("palm_%sseg1_joint" % (p), "%sseg7_%sseg8_joint" % (p,p) , 10, self.tfBuffer)
                
                
                #f.addSensor("palm_%sseg1_joint" % (p), "%sseg2_%sseg3_joint" % (p,p),  10, self.tfBuffer)
                #f.addSensor("palm_%sseg1_joint" % (p), "%sseg5_%sseg6_joint" % (p,p) , 10, self.tfBuffer)
                #f.addSensor("palm_%sseg1_joint" % (p), "%sseg8_%sseg9_joint" % (p,p) , 10, self.tfBuffer)
                
                f.addSensor("palm_%sseg1_joint" % (p), "%sseg11_%sseg12_joint" % (p,p), 10, self.tfBuffer)


    def getErrors(self):
        meanError = self.fingers[0].getTipErrors()
        return meanError 
        
    def getRadiusFromLengths(self, length1, length2): 
        if(length1 > length2):
            l1 = length1
            l2 = length2
            sign = -1
        else:
            l1 = length2
            l2 = length1
            sign = 1
        offset = 10

        R = ((l2*offset)/(l1-l2) + offset) * sign
        theta = l1/R
        return R, theta

    def getPrediction(self, dis):
        
        path = []

        for s in range(len(self.sensors)):
            sensor = self.sensors[s]
            if(s == 0):
                baseJoint = self.jointLinks[self.sensors[s].startJoint]
                outsideLength, insideLength, centerLength = sensor.outsideLength, sensor.insideLength, sensor.centerLength
            else:
                prev = self.sensors[s-1]
                baseJoint = self.jointLinks[prev.endJoint]
                outsideLength = sensor.outsideLength - prev.outsideLength
                insideLength = sensor.insideLength - prev.insideLength
                centerLength = sensor.centerLength - prev.centerLength

            endJoint = self.jointLinks[sensor.endJoint]


            if(outsideLength < insideLength):
                radius, theta = self.getRadiusFromLengths(outsideLength, centerLength)
            else:
                radius, theta = self.getRadiusFromLengths(centerLength,insideLength)

            center = baseJoint.predictedPos + baseJoint.predictedPerp * radius



            globalTheta = theta - baseJoint.predictedPerp.Angle()
            end = center + Vector2(radius*cos(pi-globalTheta), radius*sin(pi-globalTheta))

            startTheta = baseJoint.predictedPerp.Angle()
            endTheta = theta - startTheta
            interval = theta/10

            for i in range(50):
                globalTheta = pi - interval*i + startTheta
                path.append(center + Vector2(radius*cos(globalTheta), radius*sin(globalTheta)))

            endJoint.predictedPos = end
            endJoint.predictedPerp = (center-end).Normalise()* (radius/abs(radius))
            
            #dis.drawLine(end.x, end.y, end.x+(endJoint.predictedPerp.x*30), end.y+(endJoint.predictedPerp.y*30), fill = "blue")
            #dis.drawLine(endJoint.pos.x, endJoint.pos.y, endJoint.pos.x+(endJoint.perp.x*30), endJoint.pos.y+(endJoint.perp.y*30), fill = "red")
            
            #dis.drawLine(baseJoint.predictedPos.x, baseJoint.predictedPos.y, baseJoint.predictedPos.x+(baseJoint.predictedPerp.x*30), baseJoint.predictedPos.y+(baseJoint.predictedPerp.y*30), fill = "blue")
            #dis.drawLine(baseJoint.pos.x, baseJoint.pos.y, baseJoint.pos.x+(baseJoint.perp.x*30), baseJoint.pos.y+(baseJoint.perp.y*30), fill = "red")
            if showNormals.get():
                p2 = endJoint.predictedPos + endJoint.predictedPerp*20
                dis.drawLine(endJoint.predictedPos.x, endJoint.predictedPos.y, p2.x, p2.y, fill = "red")
            if showSensors.get():
                dis.drawCircle(end.x, end.y, 4, fill="yellow")
            if showCenters.get():
                dis.drawCircle(center.x, center.y, 3, fill="blue")
            if showCircles.get():
                dis.drawCircle(center.x, center.y, radius)
        
        for p in range(len(path)-1):
            #dis.drawCircle(path[p].x, path[p].y, 2)
            dis.drawLine(path[p].x, path[p].y, path[p+1].x, path[p+1].y, fill="red")

    def calcCurve2(self, dis, base):
        
        end = base
        next = self.jointLinks[base].next
        while next != None:
            self.jointLinks[end].draw(dis)
            end = next
            
            next = self.jointLinks[next].next

        perp1 = (self.jointLinks[base].inside - self.jointLinks[base].outside)*1
        perp2 = (self.jointLinks[end].inside - self.jointLinks[end].outside)*3

        t1 = getTransform("palm", self.jointLinks[base].link).transform.translation
        t2 = getTransform("palm", self.jointLinks[end].link).transform.translation

        p1 = Vector2(t1.y*1000, t1.z*1000)
        p2 = Vector2(t2.y*1000, t2.z*1000)

        p3 = p1 + perp1
        p4 = p2 + perp2

        line1 = [[p1.x, p1.y],[p3.x, p3.y]]
        line2 = [[p2.x, p2.y],[p4.x, p4.y]]
 

        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            raise Exception('lines do not intersect')

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        
        c = Vector2(x, y)

        dis.drawLine(p1.x, p1.y, p3.x, p3.y)
        dis.drawLine(p2.x, p2.y, p4.x, p4.y)


        centerLength, outsideStrain, insideStrain = self.calcStrains(dis, base)
        outsideLength = outsideStrain * centerLength
        insideLength = insideStrain * centerLength

        R, theta = self.getRadiusFromStrains(centerLength, insideLength)

        p = getTransform("palm", self.jointLinks[base].link).transform
        eulers = quaternionToEuler(p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w)
       
        p = Vector2(p.translation.y*1000, p.translation.z*1000)
        seg1Perp = Vector2(cos(eulers[0]), sin(eulers[0])).Normalise()

        c1 = p + (seg1Perp*R)
        c2 = p2 + (perp2.Normalise() * R)

        dis.drawLine(p.x, p.y, p.x+100*cos(eulers[0]), p.y+100*sin(eulers[0]))
        
        #c = p + (seg1Perp*(R))
        dis.drawCircle(p.x, p.y, 2, fill = "blue")
        dis.drawCircle(c1.x, c1.y, 2, fill = "red")
        dis.drawCircle(c2.x, c2.y, 2, fill = "red")
        end = Vector2(c.x+-R*sin(pi-theta), c.y+R*cos(pi-theta))
        dis.drawCircle(end.x, end.y, 5, fill="green")
        dis.drawCircle(c1.x, c1.y, R)
        dis.drawCircle(mouseX-500, 500-mouseY, radi)
        
    def calcCurve(self, dis, base):
        
        l1, outsideStrain, insideStrain = self.calcStrains(dis, base)
        sign = +1
        if(outsideStrain >= 1):
            l2 = l1*outsideStrain

        else:
            l2 = l1*insideStrain
            sign = -1

        offset = 10
        R = (offset*l1)/(l1-l2) * sign
        theta = l1/R
        
        p = getTransform("palm", self.jointLinks[base].link).transform
        eulers = quaternionToEuler(p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w)
        p = Vector2(p.translation.y*1000, p.translation.z*1000)
        dis.drawLine(p.x, p.y, p.x+100*cos(eulers[0]), p.y+100*sin(eulers[0]))
        
        c = p + Vector2(-R*cos(eulers[0]), -R*sin(eulers[0]))
        dis.drawCircle(c.x, c.y, 5, fill = "red")
        end = Vector2(c.x+-R*cos(pi-theta), c.y+R*sin(pi-theta))
        dis.drawCircle(end.x, end.y, 5, fill="green")
        dis.drawCircle(c.x, c.y, R)
        dis.drawCircle()

    def update(self, options):
        for f in self.fingers:
            f.update(self.tfBuffer, options)
            try:
                arr = []
                springOutputArr = ""
                effortInputsArr = ""
                for j in f.jointNames:
                    springOutputArr += "%+0.2f, " % (self.springStates[j])
                    effortInputsArr += "%+0.2f, " % (self.effortInputs[j])

                    self.jointLinks[j].force = self.effortInputs[j]+self.springStates[j]
                    #arr.append(round(self.jointStates[j]-self.springStates[j],2))
            
            except:
                print("Joint states not recieved")
        
    def draw(self, dis, options):
        for f in self.fingers:
            try:
                f.draw(dis, self.effortInputs, options)
            except KeyError:
                pass
    
        names = rospy.get_param("hand_controller/joints")
        arr = []
        for n in names:
            found = False
            for f in self.fingers:
                try:
                    col = f.colors[n]
                    found = True
                except KeyError:
                    #print("Missing:", n)
                    pass
            if found:
                for i in range(3):
                    arr.append(col[i])
            else:
                for i in range(3):
                    arr.append(0)
        
        pub = rospy.Publisher("Design/forces", Float32MultiArray, queue_size=10)
        data = Float32MultiArray(data = arr)
        pub.publish(data)

def getTransform(buffer, link1, link2,  timeout=5):

            found = False
            while found == False:
                try:
                    trans = buffer.lookup_transform(link1, link2, rospy.Time(), rospy.Duration(timeout))
                    
                    found = True
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    print("error")
                    return False
            return trans

def quaternionToEuler(x, y, z, w):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if(abs(sinp) >= 1):
        if(sinp > 0):
            pitch = pi/2
        else:
            pitch = -pi/2
    else:
        pitch = asin(sinp)
    
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def setMouse(event):
    global mouseX, mouseY
    mouseX = event.x
    mouseY = event.y

def scrollerUp(event):
    global radi
    radi += 2

def scrollerDown(event):
    global radi
    radi -= 2

def mapper(value, lower, upper, v1, v2):
    value = max(lower, min(upper, value))
    frac = (value-lower)/(upper-lower)
    output = v1 + (v2-v1)*frac
    return output




if __name__ == "__main__":
    root = Tk()
    root.bind("<Motion>", setMouse)
    root.bind("<Button-4>", scrollerUp)
    root.bind("<Button-5>", scrollerDown)

    d = display.Display(root)
    d.scale = 2

    options = {}
    labels = ["showReal", "showSensors", "showCenters", "showCircles", "showNormals", "showSides", "showExpected", "showPredicted", "showForces"]
    for l in labels:
        options[l] = BooleanVar(False)


    options["showForces"].set(True)
    Checkbutton(root, text="Show Sensors", variable=options["showSensors"]).place(x = 10, y = 20)
    Checkbutton(root, text="Show Centers", variable=options["showCenters"]).place(x = 10, y = 40)
    Checkbutton(root, text="Show Circles", variable=options["showCircles"]).place(x = 10, y = 60)
    Checkbutton(root, text="Show Normals", variable=options["showNormals"]).place(x = 10, y = 80)
    Checkbutton(root, text="Show Sides", variable=options["showSides"]).place(x = 10, y = 100)
    Checkbutton(root, text="Show Expected", variable=options["showExpected"]).place(x = 10, y = 120)
    Checkbutton(root, text="Show Predicted", variable=options["showPredicted"]).place(x = 10, y = 140)
    Checkbutton(root, text="Show Forces", variable=options["showForces"]).place(x = 10, y = 160)
    Checkbutton(root, text="Show Real", variable=options["showReal"]).place(x = 10, y=190)

    rospy.init_node('gripperController', anonymous=True)
    rospy.logerr("controller starting")

    g = Gripper()
    g.setup()

    print("Setup complete")

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        d.clear()
        g.update(options)
        #g.calcStrains(d)
        g.draw(d,options)
        d.display()
        rate.sleep()
        #print("<----------->")
        #t1 = getTransform("palm", "LTseg3").transform.rotation
        #print(quaternionToEuler(t1.x, t1.y, t1.z, t1.w))
        #print("-----------")
        #t1 = getTransform("palm", "RTseg3").transform.rotation
        #print(quaternionToEuler(t1.x, t1.y, t1.z, t1.w))

