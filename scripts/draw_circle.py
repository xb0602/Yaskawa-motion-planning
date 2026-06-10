#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node("draw_circle")
    rospy.loginfo("Node has been started")

    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        #publish cmd_vel
        msg = Twist()
        msg.linear.x = 3.0  #x is the axis represents the side of the turtle
        msg.angular.z = 2.0 #2D coordinate so angular x,y dont exist
        pub.publish(msg)
        rate.sleep()






# ubuntu@ubuntu-Nitro-AN515-57:~$ rosrun turtlesim turtlesim_node
# [INFO] [1773997086.344309034]: Starting turtlesim with node name /turtlesim
# [INFO] [1773997086.346022232]: Spawning turtle [turtle1] at x=[5,544445], y=[5,544445], theta=[0,000000]

# ubuntu@ubuntu-Nitro-AN515-57:~$ rostopic list
# /rosout
# /rosout_agg
# /turtle1/cmd_vel
# /turtle1/color_sensor
# /turtle1/pose
# ubuntu@ubuntu-Nitro-AN515-57:~$ rostopic info /turtle1/cmd_vel
# Type: geometry_msgs/Twist

# Publishers: None

# Subscribers: 
#  * /turtlesim (http://ubuntu-Nitro-AN515-57:46039/)


# ubuntu@ubuntu-Nitro-AN515-57:~$ rosmsg show geometry_msgs/Twist
# geometry_msgs/Vector3 linear
#   float64 x
#   float64 y
#   float64 z
# geometry_msgs/Vector3 angular
#   float64 x
#   float64 y
#   float64 z

# ubuntu@ubuntu-Nitro-AN515-57:~$ rostopic info /turtle1/cmd_vel
# Type: geometry_msgs/Twist

# Publishers: 
#  * /draw_circle (http://ubuntu-Nitro-AN515-57:43387/)

# Subscribers: 
#  * /turtlesim (http://ubuntu-Nitro-AN515-57:46039/)

# ubuntu@ubuntu-Nitro-AN515-57:~$ rosrun my_robot_controller draw_circle.py 
# [INFO] [1773998334.536573]: Node has been started
