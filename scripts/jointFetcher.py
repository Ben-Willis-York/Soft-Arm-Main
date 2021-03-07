#!/usr/bin/env python

import tf
from Tkinter import *
import rospy, os, time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import geometry_msgs.msg
import gazebo_msgs.msg
from gazebo_msgs.msg import LinkStates

import roslib; roslib.load_manifest('urdfdom_py')
from urdf_parser_py.urdf import URDF


if __name__ == '__main__':
    rospy.loginfo("Fetching Joints")
    rospy.init_node('joint_fetcher', anonymous=True)
    
    t = tf.TransformerROS(True, rospy.Duration(1))
    t.getFrameStrings()

    jointNames = rospy.get_param("/arm_controller/joints")
    linkNames = []

    jointIndex = []

    robot = URDF.from_xml_string(rospy.get_param("/robot_description"))

    data = rospy.wait_for_message("joint_states", JointState)

    rospy.wait_for_service("gazebo/get_link_state")
    d = rospy.ServiceProxy("gazebo/get_link_state", LinkStates)
    print("Done")
    print(d("arm1_1", "arm2_1"))

    #transforms = rospy.wait_for_message("tf", tf.TFMessage)
    #print(transforms)

    '''
    for n in jointNames:
        for i in range(len(data.name)):
            if(n == data.name[i]):
                jointIndex.append(i)
                #pos = [data.position[jointIndex[i]*3], data.positions[jointIndex[i]*3 + 1], data.positions[jointIndex[i]*3 + 2]]
                #print(n, pos)

    print(len(data.position))
    print(len(data.name))

    print(jointNames)
    '''

    for n in jointNames:
        for j in robot.joints:
            if(j.name == n):
                print(j.name, j.limit.upper, j.limit.lower, j.parent)
                if j.parent not in linkNames:
                    linkNames.append(j.parent)
                if j.child not in linkNames:
                    linkNames.append(j.child)
                
    for i in range(1,len(linkNames)):
        m = t.waitForTransform(linkNames[i-1], linkNames[i], rospy.Time(), rospy.Duration(10))
        print(m)

    '''
    for n in linkNames:
        for l in robot.links:
            if(l.name == n):
                print(l)
    '''
    try:
        pass
    except rospy.ROSInterruptException:
        pass
