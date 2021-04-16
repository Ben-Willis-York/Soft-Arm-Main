#!/usr/bin/env python

import gripperVisualiser
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

class GripperControlPanel:
    def __init__(self):
        self.root = Tk()
        self.root.geometry("600x800")
        self.display = display.Display(self.root, h=500, w=600, pos=Vector2(0,300) )
        self.display.scale = 1.8
        self.display.setScreenOrigin(0.5,0.8)

        self.options = {}
        self.setup()

        self.gripperSolver = gripperVisualiser.Gripper()
        self.gripperSolver.setup()

    def setup(self):
        #options = {}
        labels = ["showReal", "showSensors", "showCenters", "showCircles", "showNormals", "showSides", "showExpected", "showPredicted", "showForces"]
        for l in labels:
            self.options[l] = BooleanVar(False)


        self.options["showForces"].set(True)
        Checkbutton(self.root, text="Show Sensors", variable=self.options["showSensors"]).place(x = 10, y = 20)
        Checkbutton(self.root, text="Show Centers", variable=self.options["showCenters"]).place(x = 10, y = 40)
        Checkbutton(self.root, text="Show Circles", variable=self.options["showCircles"]).place(x = 10, y = 60)
        Checkbutton(self.root, text="Show Normals", variable=self.options["showNormals"]).place(x = 10, y = 80)
        Checkbutton(self.root, text="Show Sides", variable=self.options["showSides"]).place(x = 10, y = 100)
        Checkbutton(self.root, text="Show Expected", variable=self.options["showExpected"]).place(x = 10, y = 120)
        Checkbutton(self.root, text="Show Predicted", variable=self.options["showPredicted"]).place(x = 10, y = 140)
        Checkbutton(self.root, text="Show Forces", variable=self.options["showForces"]).place(x = 10, y = 160)
        Checkbutton(self.root, text="Show Real", variable=self.options["showReal"]).place(x = 10, y=190)


    def update(self):
        self.display.clear()
        self.gripperSolver.update(self.options)
        self.gripperSolver.draw(self.display, self.options)
        self.display.display()



rospy.init_node('gripperControlPanel', anonymous=True)
rospy.logerr("controller starting")





panel = GripperControlPanel()
rate = rospy.Rate(100)

while not rospy.is_shutdown():
    #d.clear()
    panel.update()
    #g.calcStrains(d)
    #g.draw(d)
    #d.display()
    rate.sleep()
    #print("<------