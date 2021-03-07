#!/usr/bin/env python

from Tkinter import *
import rospy, os
from std_msgs.msg import Float64MultiArray
import std_msgs.msg

import roslib; roslib.load_manifest('urdfdom_py')
from urdf_parser_py.urdf import URDF

descripton = rospy.get_param("/robot_description")

path = os.path.dirname(os.path.dirname(__file__))

print(path)

file = open(path+"/urdf/description_log.urdf", "w")

file.write(descripton)
file.close()