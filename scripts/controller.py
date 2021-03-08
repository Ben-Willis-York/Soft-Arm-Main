#!/usr/bin/env python
import display, robot, math, time, VectorClass, simulator, controller
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

class Controller():
    def __init__(self):
        self.names = []
        self.setpoints = []
        self.currentSetpoints = []
        self.path = []

        self.state = []

        self.speed = [3,3,3]

        self.names = rospy.get_param("/arm_controller/joints")
        for n in self.names:
            self.setpoints.append(0)
            self.currentSetpoints.append(0)
        
    def setPath(self, path):
        #self.path = path
        self.setpoints = path[0]
        self.path = path[1:]

    def setJointStates(self, angles):
        pub = rospy.Publisher("arm_controller/command", Float64MultiArray, queue_size=10)
        armNames = rospy.get_param("/arm_controller/joints")
        data = rospy.wait_for_message("joint_states", JointState)
        arr = []
        for n in armNames:
            for d in range(len(data.name)):
                if(data.name[d] == n):
                    arr.append(data.position[d])
        
        arr[1] = -radians(angles[0])
        arr[2] = -radians(angles[1])
        arr[3] = -radians(angles[2])
        packet = Float64MultiArray(data = arr)
        pub.publish(packet)

        arr = arr[1:]
        for a in range(len(angles)):
            self.currentSetpoints[a] = angles[a]

    def getJointStates(self):
        return self.state

    def getSetpoints(self):
        return self.setpoints
    
    def setSetpoints(self, angles):
        for a in range(len(angles)):
            self.setpoints[a] = angles[a]

    def update(self):
        armNames = rospy.get_param("/arm_controller/joints")
        data = rospy.wait_for_message("joint_states", JointState)
        arr = []
        for n in armNames:
            for d in range(len(data.name)):
                if(data.name[d] == n):
                    arr.append(-data.position[d])
        arr = arr[1:]
        self.state = arr
        '''
        nextStep = []
        for a in range(len(self.state)):
            nextStep.append(0)
            difference = self.currentSetpoints[a] - self.setpoints[a]
            #print(difference)
            if(difference > 5):
                nextStep[a] = self.currentSetpoints[a] - self.speed[a]
            elif(difference < -5):
                nextStep[a] = self.currentSetpoints[a] + self.speed[a]
            else:
                nextStep[a] = self.setpoints[a]
        self.setJointStates(nextStep)
        '''

        ready = True
        for a in range(len(self.state)):
            difference = abs(degrees(self.state[a]) - self.setpoints[a])
            if(difference >= 20):
                print("NOT READY:", degrees(self.state[a]), self.setpoints[a])
                ready = False
        if(ready):
            if(len(self.path) > 0):
                self.setSetpoints(self.path[0])
                self.setJointStates(self.setpoints)
                self.path = self.path[1:]
            else:
                pass
                #nextStep[a] = self.setpoints[a]

                



