import display, robot, math, time, VectorClass, simulator, controller
from VectorClass import *
from Tkinter import *
from math import *
import tf2_py
import tf2_ros
import rospy, os, time, math
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import geometry_msgs.msg

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
        

    def draw(self, dis):
        transform = getTransform("palm", self.link).transform
        pos = Vector2(transform.translation.y*1000, transform.translation.z*1000)

        dir = Vector2(0,0)

        if(self.next):
            t1 = getTransform("palm", jointLinks[self.next].link).transform
            p1 = Vector2(t1.translation.y*1000, t1.translation.z*1000)
            dir += p1-pos
        if(self.previous):
            t1 = getTransform("palm", jointLinks[self.previous].link).transform
            p1 = Vector2(t1.translation.y*1000, t1.translation.z*1000)
            dir += pos-p1

        dir = dir.Normalise()
        perp = Vector2(dir.y, -dir.x)

        right = pos + perp*10
        left = pos - perp*10
        #dis.drawLine(p1.x, p1.y, p1.x +dir.x, p1.y+dir.y)
        dis.drawCircle(pos.x, pos.y, 5)
        dis.drawCircle(right.x, right.y, 2, fill = "red")
        dis.drawCircle(left.x, left.y, 2, fill = "green")



def getTransform(link1, link2, timeout=5):

            found = False
            while found == False:
                try:
                    trans = tfBuffer.lookup_transform(link1, link2, rospy.Time(), rospy.Duration(timeout))
                    
                    found = True
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    print("error")
                    return False
            return trans


def getJoint(name):
    for j in joints:
        if(j.name == name):
            return j
    return False


class Gripper(object):
    def __init__(self):
        self.jointLinks = []
        

root = Tk()
d = display.Display(root)
d.scale = 3

rospy.init_node('gripperController', anonymous=True)
rospy.logerr("controller starting")

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

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

