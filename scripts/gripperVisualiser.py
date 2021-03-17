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
from gazebo_msgs.msg import ApplyJointEffort

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


class JointLink(object):
    def __init__(self, name):
        self.name = name
        self.previous = None
        self.next = None
        self.link = None
        self.pos = Vector2(0,0)
        self.inside = Vector2(0,0)
        self.outside = Vector2(0,0)
        self.insideStrain = 0
        self.outsideStrain = 0
        
    def update(self, jointLinks):
        transform = getTransform("palm", self.link).transform
        self.pos = Vector2(transform.translation.y*1000, transform.translation.z*1000)

        if(self.previous == None):
            dir = Vector2(0,1)
        else:
            dir = Vector2(0,0)
            if(self.next):
                t1 = getTransform("palm", jointLinks[self.next].link).transform
                p1 = Vector2(t1.translation.y*1000, t1.translation.z*1000)
                dir += p1-self.pos

            if(self.previous):
                t1 = getTransform("palm", jointLinks[self.previous].link).transform
                p1 = Vector2(t1.translation.y*1000, t1.translation.z*1000)
                
                dir += self.pos-p1

        dir = dir.Normalise()
        rot = transform.rotation
        eulers = quaternionToEuler(rot.x, rot.y, rot.z, rot.w)

        perp = Vector2(dir.y*cos(eulers[2]), -dir.x*cos(eulers[2]))
        self.inside = self.pos + perp*10
        self.outside = self.pos - perp*10

    def draw(self, dis):
        #dis.drawLine(p1.x, p1.y, p1.x +dir.x, p1.y+dir.y)
        dis.drawCircle(self.pos.x, self.pos.y, 5)
        dis.drawCircle(self.outside.x, self.outside.y, 2)
        dis.drawCircle(self.inside.x, self.inside.y, 2)




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
        print(self.fingerBases)

    def calcStrains(self, dis, base):
        
        outsideLength = 0
        insideLength = 0
        centerLength = 0
        visited = []

        joint = base
        next = self.jointLinks[joint].next
        while next != None:
            outside = self.jointLinks[next].outside - self.jointLinks[joint].outside
            inside = self.jointLinks[next].inside - self.jointLinks[joint].inside
            center = self.jointLinks[next].pos - self.jointLinks[joint].pos
            outsideLength += outside.Mag()
            insideLength += inside.Mag()   
            centerLength += center.Mag()
            visited.append(joint)
            joint = next
            next = self.jointLinks[next].next 
        
        outsideFrac = outsideLength/centerLength
        outsideCol = '#%02x%02x%02x' % (int(outsideFrac*127), 127, int(127/outsideFrac))

        insideFrac = insideLength/centerLength
        insideCol = '#%02x%02x%02x' % (int(insideFrac*127), 127, int(127/insideFrac))

        for v in visited:
            p1 = self.jointLinks[v].inside
            p2 = self.jointLinks[self.jointLinks[v].next].inside
            dis.drawLine(p1.x, p1.y, p2.x, p2.y, fill=insideCol)
            p1 = self.jointLinks[v].outside
            p2 = self.jointLinks[self.jointLinks[v].next].outside
            dis.drawLine(p1.x, p1.y, p2.x, p2.y, fill=outsideCol)

        return centerLength, outsideFrac, insideFrac
        
    def calcCurve(self, dis, base):
        
        l1, outsideStrain, insideStrain = self.calcStrains(dis, base)
        sign = +1
        if(outsideStrain >= 1):
            l2 = l1*outsideStrain
            print("out")
        else:
            l2 = l1*insideStrain
            sign = -1
            print("in")
        offset = 10
        R = (offset*l1)/(l1-l2) * sign
        theta = l1/R
        
        p = getTransform("palm", self.jointLinks[base].link).transform
        eulers = quaternionToEuler(p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.z)
        p = Vector2(p.translation.y*1000, p.translation.z*1000)
        

        c = p + Vector2(-R*cos(eulers[2]), 0)
        dis.drawCircle(c.x, c.y, 5, fill = "red")
        end = Vector2(c.x+-R*cos(pi-theta), c.y+R*sin(pi-theta))
        dis.drawCircle(end.x, end.y, 5, fill="green")

    def applySpring(self, name):
        print("waiting for service")
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        #try:
        applyEffort = rospy.ServiceProxy('/gazebo/apply_joint_effort', LinkState)
        data = ApplyJointEffort("{joint_name: %s, effort: 2, start_time: %s, duration: %s}" % (name, 0, 1000000000))
        applyEffort(data)
        #except:
        #    print("error")
        #    pass


    def update(self):
        for j in self.jointLinks.values():
            j.update(self.jointLinks)
        for j in self.jointNames:
            self.applySpring(j)

    def draw(self, dis):
        for b in self.fingerBases[1:2]:
            self.calcCurve(dis, b)
        #self.calcCurve(dis)
        #for j in self.jointLinks.values():
        #    j.draw(dis)
        
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

root = Tk()
d = display.Display(root)
d.scale = 3

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

