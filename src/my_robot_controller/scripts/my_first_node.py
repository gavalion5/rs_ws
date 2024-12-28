#!/usr/bin/env python3
import rospy

if __name__ == "__main__":
    rospy.init_node("test_node")
    rospy.loginfo("hello from test node")

    rate = rospy.Rate(10)
    i = 0
    while not rospy.is_shutdown():
        rospy.loginfo("hello")
        rate.sleep()