import rospy, os, time, random
from std_msgs.msg import Float32MultiArray

rospy.init_node('testNode', anonymous=True)
rospy.loginfo("controller starting")
rate = rospy.Rate(10)

pub = rospy.Publisher("Design/forces", Float32MultiArray, queue_size=10)

n = 0


#print(type(rospy.get_param("arm_controller/Rev1/pid/i")))
while True:
    arr = []
    for i in range(48):
        for a in range(3):
            arr.append(random.uniform(0,1))
    
    data = Float32MultiArray(data = arr)
    pub.publish(data)
    print("Published")
    n+=1
    rate.sleep()