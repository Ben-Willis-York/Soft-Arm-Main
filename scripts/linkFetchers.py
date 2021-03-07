#!/usr/bin/env python

import tf2_py
import tf2_ros
from Tkinter import *
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
            print(trans)
            found = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("error")
            return False
    return trans

def getLink(name):
    for l in links:
        if(l.name == name):
            return l
            



if __name__ == '__main__':
    rospy.init_node("tfListener")

    

    while not rospy.is_shutdown():
        rate.sleep