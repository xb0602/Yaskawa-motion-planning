#!/usr/bin/env python3
import sys
import rospy
import copy
import moveit_commander
from moveit_commander.move_group import MoveGroupCommander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from math import pi, cos, sin
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def draw_path_marker(waypoints, publisher):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "spline_path"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    marker.scale.x = 0.005 

    # Color: Green for this curved path
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    for wp in waypoints:
        p = Point()
        p.x = wp.position.x
        p.y = wp.position.y
        p.z = wp.position.z
        marker.points.append(p)

    publisher.publish(marker)

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("Linear_Spline", anonymous=True)

    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    group_name = "manipulator"
    move_group: MoveGroupCommander = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_pose_reference_frame("world")

    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)

    # target_x = 0.2
    # target_y = -0.1
    # target_z = 0.65
    # rospy.loginfo("Planning trajectory to absolute position [X: %f, Y: %f, Z: %f]", target_x, target_y, target_z)
    # # Use set_position_target instead of set_pose_target.
    # # This allows MoveIt to automatically calculate the best wrist orientation.
    # move_group.set_position_target([target_x, target_y, target_z])
    # # Plan and execute the trajectory
    # move_group.go(wait=True)
    # move_group.stop()
    # rospy.sleep(1.0)

    rospy.loginfo("1. Executing MOVJ to a comfortable starting posture...")
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = math.radians(0)
    joint_goal[1] = math.radians(0)   # Vươn vai tới trước
    joint_goal[2] = math.radians(0)  # Gập nhẹ khuỷu tay
    joint_goal[3] = math.radians(20)
    joint_goal[4] = math.radians(0)  # Chúc mũi gắp xuống
    joint_goal[5] = math.radians(0)

    move_group.go(joint_goal, wait=True)
    move_group.stop()
    rospy.sleep(1.0)

    rospy.loginfo("2. Calculating circular path...")
    waypoints = []
    wpose = move_group.get_current_pose().pose

    radius = 0.05
    center_x = wpose.position.x
    center_y = wpose.position.y + radius

    resolution = 20 # Tăng độ phân giải lên 20 điểm để đường cong mượt hơn

    for i in range(resolution + 1):
        angle = (pi / resolution) * i
        
        wpose.position.x = center_x + radius * cos(angle - pi/2)
        wpose.position.y = center_y + radius * sin(angle - pi/2)
        
        waypoints.append(copy.deepcopy(wpose))

    draw_path_marker(waypoints, marker_pub)

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   
                                       0.01,        
                                       True)        

    if fraction > 0.9:
        rospy.loginfo("Path planned successfully (Fraction: %f). Executing...", fraction)
        move_group.execute(plan, wait=True)
    else:
        rospy.logwarn("Could not compute the full path. Fraction: %f", fraction)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass