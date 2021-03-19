import display, robot, math, time, VectorClass, simulator, controller
from VectorClass import *
from Tkinter import *
from math import *
import tf2_py
import tf2_ros
import roslib
import rospy, os, time, math
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion

import roslib; roslib.load_manifest('urdfdom_py')
from urdf_parser_py.urdf import URDF

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
    def __init__(self, startJoint, endJoint, offset, jointLinks):
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
            
        for j in self.joints:
            print(j.name)

    def getPositions(self):
        startTransform = getTransform("palm", self.startJoint).transform
        self.startCenter = Vector2(startTransform.y*1000, startTransform.z*1000)

        dir = Vector2(0,0)
        if(jointLinks[startJoint].next):
            t1 = getTransform("palm", jointLinks[jointLinks[startJoint].next].link).transform
            p1 = Vector2(t1.translation.y*1000, t1.translation.z*1000)
            dir += p1-self.pos


        endTransform = getTransform("palm", self.endJoint).transform
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

    def draw(self, dis):
        for p in self.outsidePositions:
            dis.drawCircle(p.x, p.y, 2, fill = "red")
        for p in self.insidePositions:
            dis.drawCircle(p.x, p.y, 2, fill = "green")




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

        self.predictedPos = Vector2(0,0)
        self.predictedDir = Vector2(0,0)
        self.predictedPerp = Vector2(0,0)
        
    def update(self, jointLinks):
        transform = getTransform("palm", self.link).transform
        self.pos = Vector2(transform.translation.y*1000, transform.translation.z*1000)

        #if(self.previous == None):
        #    dir = Vector2(0,1)

        #else:
        dir = Vector2(0,0)
        if(self.next):
            t1 = getTransform("palm", jointLinks[self.next].link).transform
            p1 = Vector2(t1.translation.y*1000, t1.translation.z*1000)
            dir += p1-self.pos

        if(self.previous):
            t1 = getTransform("palm", jointLinks[self.previous].link).transform
            p1 = Vector2(t1.translation.y*1000, t1.translation.z*1000)
            
            dir += self.pos-p1

        self.dir = dir.Normalise()
        rot = transform.rotation
        eulers = quaternionToEuler(rot.x, rot.y, rot.z, rot.w)

        self.perp = Vector2(-dir.y*cos(eulers[2]), dir.x*cos(eulers[2])).Normalise() * -1
        self.inside = self.pos + self.perp*10
        self.outside = self.pos - self.perp*10

    def draw(self, dis, jointLinks):
        #dis.drawLine(p1.x, p1.y, p1.x +dir.x, p1.y+dir.y)
        #dis.drawCircle(self.pos.x, self.pos.y, 2)
        if(self.next != None):
            dis.drawLine(self.pos.x, self.pos.y, 
                        jointLinks[self.next].pos.x, jointLinks[self.next].pos.y, fill="blue")
        #dis.drawCircle(self.predictedPos.x, self.predictedPos.y, 2, fill = "blue")
        #dis.drawCircle(self.outside.x, self.outside.y, 2)
        #dis.drawCircle(self.inside.x, self.inside.y, 2)




def getJoint(name):
    for j in joints:
        if(j.name == name):
            return j
    return False


class Gripper(object):
    def __init__(self):
        self.jointNames = rospy.get_param("/hand_controller/joints")
        self.robot = URDF.from_xml_string(rospy.get_param("/robot_description"))
        self.link_map = self.robot.link_map
        self.joint_map = self.robot.joint_map
        self.jointLinks = {}
        self.fingerBases = []
        self.sensors = []

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
        #print(self.fingerBases)

        self.sensors.append(StrainSensor("palm_LTseg1_joint", "LTseg3_LTseg4_joint", 10, self.jointLinks))
        self.sensors.append(StrainSensor("palm_LTseg1_joint", "LTseg7_LTseg8_joint", 10, self.jointLinks))
        self.sensors.append(StrainSensor("palm_LTseg1_joint", "LTseg9_LTseg10_joint", 10, self.jointLinks))
        self.sensors.append(StrainSensor("palm_LTseg1_joint", "LTseg11_LTseg12_joint", 10, self.jointLinks))

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

    def getPridiction(self, dis):
        
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
            #dis.drawCircle(center.x, center.y, 3, fill="blue")
            #dis.drawCircle(center.x, center.y, radius)

            globalTheta = theta - baseJoint.predictedPerp.Angle()
            end = center + Vector2(radius*cos(pi-globalTheta), radius*sin(pi-globalTheta))

            startTheta = baseJoint.predictedPerp.Angle()
            endTheta = theta - startTheta
            interval = theta/10

            for i in range(10):
                globalTheta = pi - interval*i + startTheta
                path.append(center + Vector2(radius*cos(globalTheta), radius*sin(globalTheta)))

            endJoint.predictedPos = end
            endJoint.predictedPerp = (center-end).Normalise()* (radius/abs(radius))
            
            #dis.drawLine(end.x, end.y, end.x+(endJoint.predictedPerp.x*30), end.y+(endJoint.predictedPerp.y*30), fill = "blue")
            #dis.drawLine(endJoint.pos.x, endJoint.pos.y, endJoint.pos.x+(endJoint.perp.x*30), endJoint.pos.y+(endJoint.perp.y*30), fill = "red")
            
            #dis.drawLine(baseJoint.predictedPos.x, baseJoint.predictedPos.y, baseJoint.predictedPos.x+(baseJoint.predictedPerp.x*30), baseJoint.predictedPos.y+(baseJoint.predictedPerp.y*30), fill = "blue")
            #dis.drawLine(baseJoint.pos.x, baseJoint.pos.y, baseJoint.pos.x+(baseJoint.perp.x*30), baseJoint.pos.y+(baseJoint.perp.y*30), fill = "red")

            dis.drawCircle(end.x, end.y, 4, fill="yellow")
        
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
        print(end)

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
        print(eulers)
        p = Vector2(p.translation.y*1000, p.translation.z*1000)
        dis.drawLine(p.x, p.y, p.x+100*cos(eulers[0]), p.y+100*sin(eulers[0]))
        
        c = p + Vector2(-R*cos(eulers[0]), -R*sin(eulers[0]))
        dis.drawCircle(c.x, c.y, 5, fill = "red")
        end = Vector2(c.x+-R*cos(pi-theta), c.y+R*sin(pi-theta))
        dis.drawCircle(end.x, end.y, 5, fill="green")
        dis.drawCircle(c.x, c.y, R)
        dis.drawCircle()

    def update(self):
        for j in self.jointLinks.values():
            j.update(self.jointLinks)
        for j in self.fingerBases:
            self.jointLinks[j].predictedPerp = self.jointLinks[j].perp
            self.jointLinks[j].predictedPos = self.jointLinks[j].pos

        for s in self.sensors:
            s.update(self.jointLinks)
        
    def draw(self, dis):
        self.getPridiction(dis)
        #for s in self.sensors:
            #s.draw(dis)
        for next in self.fingerBases[1:2]:
            while next != None:
                self.jointLinks[next].draw(dis, self.jointLinks)
                next = self.jointLinks[next].next
            #self.calcCurve2(dis, b)
        #self.calcCurve(dis)
        #for j in self.jointLinks.values():
            #j.draw(dis)
        
def getTransform(link1, link2,  timeout=5):

            found = False
            while found == False:
                try:
                    trans = tfBuffer.lookup_transform(link1, link2, rospy.Time(), rospy.Duration(timeout))
                    
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


radi = 50
mouseX, mouseY = 0, 0    
root = Tk()
root.bind("<Motion>", setMouse)
root.bind("<Button-4>", scrollerUp)
root.bind("<Button-5>", scrollerDown)
d = display.Display(root)
d.scale = 2

rospy.init_node('gripperController', anonymous=True)
rospy.logerr("controller starting")

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

g = Gripper()
g.setup()

print("Setup complete")

while True:
    d.clear()
    g.update()
    #g.calcStrains(d)
    g.draw(d)
    d.display()
    #print("<----------->")
    #t1 = getTransform("palm", "LTseg3").transform.rotation
    #print(quaternionToEuler(t1.x, t1.y, t1.z, t1.w))
    #print("-----------")
    #t1 = getTransform("palm", "RTseg3").transform.rotation
    #print(quaternionToEuler(t1.x, t1.y, t1.z, t1.w))




joints = []
links = []

jointLinks = {}

jointNames = rospy.get_param("/hand_controller/joints")
robot = URDF.from_xml_string(rospy.get_param("/robot_description"))

link_map = robot.link_map
joint_map = robot.joint_map

for j in jointNames:
    joints.append(robot.joint_map[j])

    jointLinks[j] = JointLink(j)
    jointLinks[j].link = robot.joint_map[j].child

    #print(joints[-1].child)
    if(robot.link_map[joints[-1].child] not in links):
        links.append(robot.link_map[joints[-1].child])
    if(robot.link_map[joints[-1].parent] not in links):
        links.append(robot.link_map[joints[-1].parent])

for j in jointLinks.keys():
    link = jointLinks[j].link
    for i in jointLinks.keys():
        if(joint_map[i].child == joint_map[j].parent):
            jointLinks[j].previous = i
        if(joint_map[i].parent == link):
            jointLinks[j].next = i






j = "palm_LTseg1_joint"
while j != None:

    output = jointLinks[j].name
    if(jointLinks[j].previous):
        output += "  ," + jointLinks[j].previous
    if(jointLinks[j].next):
        output += "  ," + jointLinks[j].next
    print(output)

    j = jointLinks[j].next

#print(robot.joint_map)

#print(jointNames)

for l in links:
    transform = getTransform("palm", l.name)
    

maxEffort = 1

while True:
    root.update()
    d.clear()


    efforts = []
    jointState = rospy.wait_for_message("joint_states", JointState)

    for j in joints:
        for i in range(len(jointState.name)):
            if(j.name == jointState.name[i]):
                efforts.append(jointState.effort[i])
                effort = jointState.effort[i]
                break
        parent = j.parent
        child = j.child
        
        
    
        t1 = getTransform("palm", j.parent).transform
        t2 = getTransform("palm", j.child).transform
        p1 = t1.translation
        p2 = t2.translation
        r1 = t1.rotation
        r2 = t2.rotation
        p1 = Vector2(p1.y*1000, p1.z*1000)
        p2 = Vector2(p2.y*1000, p2.z*1000)

        if(j.name == "LTseg1_LTseg2_joint"):
            #print(r2)
            pass

        if(effort != 0):
            mag = math.log(abs(effort*1000))
        else:
            mag = 0
        mag = float(mag)/float(maxEffort)

        r = max(0.0, min(255.0, mag*255.0))
        b = max(-1.0, min(0.0, mag)) * -255.0

        col = '#%02x%02x%02x' % (int(r), 0, int(b))
        #d.drawLine(p1.x, p1.y, p2.x, p2.y, fill=col)
        #d.drawCircle(p2.x, p2.y, 3, fill = col)
        
        jointLinks[j.name].draw(d)
    #for l in links:
        #trans = getTransform("palm", l.name)
        #print trans
        #d.drawCircle(trans.transform.translation.y*1000, trans.transform.translation.z*1000, 5)
    d.display()
#     for j in jointNames:

#         parent = getJoint(j.parent.name)
#         child = getJoint(j.child.name)

#         if(parent != False and child != False):
#             parent.child = child
#             child.parent = parent
#         elif(parent != False):
#             child = Joint(j.child)

