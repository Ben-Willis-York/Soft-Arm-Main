import rospy, os, time
from std_msgs.msg import Float32MultiArray

rospy.init_node('testNode', anonymous=True)
rospy.loginfo("controller starting")
rate = rospy.Rate(5)

pub = rospy.Publisher("Design/commands", Float32MultiArray, queue_size=10)

n = 0

while True:
    arr = []
    for i in range(10):
        arr.append(n)
    data = Float32MultiArray(data = arr)
    pub.publish(data)
    n+=1
    rate.sleep()