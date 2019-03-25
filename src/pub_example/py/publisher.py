#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose2D
from graphical_client.msg import Pose2D_Array

def init_pose():
    pose = Pose2D()
    pose.x = 0
    pose.y = 0
    pose.theta = 0
    return pose

def talker():
    pub = rospy.Publisher('/trajectory', Pose2D_Array, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    aux = 1
    while not rospy.is_shutdown():
        arr = Pose2D_Array()
        for i in range(10):
            pose = init_pose()
            pose.x = 100 * ( i + 1 ) 
            pose.y = 150 * ( i + 1 ) * aux
            pose.theta +=0.7853 * i
            arr.poses.append(pose)
            aux *= -1
        print "The array is:", arr
        pub.publish(arr)
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass