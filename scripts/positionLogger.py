import display, robot, math, time, VectorClass, simulator, controller, random
from Tkinter import *
from math import *
import tf2_py
import tf2_ros
import rospy, os, time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import geometry_msgs.msg

import roslib; roslib.load_manifest('urdfdom_py')
from urdf_parser_py.urdf import URDF


if __name__ == '__main__':
    rospy.init_node('Position Logger', anonymous=True)
    
    rate = rospy.Rate(50)
    panel = ControlPanel()
    panel.setup()

    while not rospy.is_shutdown():
        panel.update()
        rate.sleep()