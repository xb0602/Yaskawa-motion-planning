#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen

previous_x = 0.0
previous_y = 0.0

def call_setpen_service(r, g, b, width, off):
    try:
        set_pen = rospy.ServiceProxy("/turtle1/set_pen", SetPen)
        response = set_pen(r, g, b, width, off)
        rospy.loginfo(response)
    except rospy.ServiceException as e:
        rospy.logwarn(e)
        

def pose_callback(msg: Pose):
    cmd = Twist()
    if msg.x > 9.0 or msg.x < 2.0 or msg.y > 9.0 or msg.y < 2.0:
        cmd.linear.x = 1.0
        cmd.angular.z = 2.0
    else:
        cmd.linear.x = 5.0
        cmd.angular.z = 0.0
    pub.publish(cmd)

    a = 5.5
    global previous_x, previous_y
    # (msg.x >= a and msg.y >= a) : Kiểm tra xem bây giờ có đang ở trong vùng đó không.
    # (previous_x < a or previous_y < a) : Kiểm tra xem trước đó có phải ít nhất một trong hai tọa độ nằm ngoài vùng này không.
    if (msg.x >= a and msg.y >= a) and (previous_x < a  or previous_y < a):
        rospy.loginfo("Set color to Red")
        call_setpen_service(255, 0, 0, 3, 0)
    elif (msg.x >= a and msg.y < a) and (previous_x < a or previous_y >= a):
        rospy.loginfo("Set color to Green")
        call_setpen_service(0, 255, 0, 3, 0)
    elif (msg.x < a and msg.y >= a) and (previous_x >= a or previous_y < a):
        rospy.loginfo("Set color to Blue")
        call_setpen_service(0, 0, 255, 3, 0)
    elif (msg.x < a and msg.y < a) and (previous_x >= a or previous_y >= a):
        rospy.loginfo("Set color to Yellow")
        call_setpen_service(255, 255, 0, 3, 0)
    
    previous_x = msg.x
    previous_y = msg.y


if __name__ == '__main__':
    rospy.init_node("turtle_controller")
    rospy.wait_for_service("/turtle1/set_pen")
    # previous_x = 0.0      initial coordinate of the turtle(avoid change color for first time)
    # previous_y = 0.0
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    rospy.loginfo("Node has been started.")
    rospy.spin()