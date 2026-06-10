#!/usr/bin/env python3
import rospy

if __name__ == '__main__':
    rospy.init_node("test_node")

    rospy.loginfo("Hello world")
    rospy.logwarn("This is a warning")
    rospy.logerr("This is an error")

    # rospy.sleep(1.0)

    # rospy.loginfo("End the program")

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        for i in range(2):
            rospy.loginfo("Suzy")
            rate.sleep()
        rospy.loginfo("Always for Suzy")
        rate.sleep()