#!/usr/bin/env python


import display, robot, math, time, VectorClass, simulator, controller
from Tkinter import *

import tf2_py
import tf2_ros
import rospy, os, time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import geometry_msgs.msg

import roslib; roslib.load_manifest('urdfdom_py')
from urdf_parser_py.urdf import URDF


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




if __name__ == '__main__':
    print("YO")
    
    rospy.init_node('newController', anonymous=True)
    rospy.logerr("controller starting")
    rate = rospy.Rate(50)

    jointNames = rospy.get_param("/arm_controller/joints")
    robot = URDF.from_xml_string(rospy.get_param("/robot_description"))


    #jointNames = ['base_link', 'arm1_1', 'arm2_1', 'arm3_3']
    #print(jointNames)

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

    print("Joints:")
    for j in joints:
        print(j.name) 

    print()

    root = Tk()
    d = display.Display(root)
    #r = robot.Robot()
    s = simulator.Sim()

    links = links[2:]

    for l in links:
        print(l.length, math.radians(math.degrees(l.joint.limits[0])), math.degrees(l.joint.limits[1]))
        len = l.length
        joint = l.joint
        lim = [math.degrees(-joint.limits[1]), math.degrees(-joint.limits[0])]
        s.addLink(l.length, limits=lim)



    c = controller.Controller(s)


    while not rospy.is_shutdown():
        try:
            s.update(0.1)
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
