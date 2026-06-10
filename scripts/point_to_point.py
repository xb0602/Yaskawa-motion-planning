#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from moveit_commander.move_group import MoveGroupCommander
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def draw_path_marker(waypoints, publisher):
    # Create a Marker object
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "robot_path"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    # Set the width of the line (in meters)
    marker.scale.x = 0.005 

    # Set the color of the line (Red, 100% opacity)
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Add waypoints to the marker
    for wp in waypoints:
        p = Point()
        p.x = wp[0]
        p.y = wp[1]
        p.z = wp[2]
        marker.points.append(p)

    # Publish the marker to RViz
    publisher.publish(marker)
    rospy.loginfo("Path marker published to RViz.")

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("Point_to_Point_Sequence", anonymous=True)

    # Setup publisher for the marker
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    group_name = "manipulator"
    move_group: MoveGroupCommander = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_pose_reference_frame("world")

    # Set speed and acceleration scaling to 10% for smooth observation
    move_group.set_max_velocity_scaling_factor(0.05)
    move_group.set_max_acceleration_scaling_factor(0.05)
    
    # # (C++) Cho phép thời gian thực thi quỹ đạo được sai số (kéo dài) thêm 200% so với tính toán
    # move_group.set_trajectory_execution_allowed_execution_duration_scaling(2.0)
    # Directly modify ROS parameters to allow the Gazebo controller more time to finish the trajectory
    rospy.set_param('/move_group/trajectory_execution/allowed_execution_duration_scaling', 5.0)
    rospy.set_param('/move_group/trajectory_execution/allowed_goal_duration_margin', 2.0)

    # RELAX GOAL TOLERANCES TO COMPENSATE FOR GAZEBO PHYSICS AND GRAVITY
    # This is the standard fix for "TIMED_OUT" steady-state errors in Gazebo
    move_group.set_goal_position_tolerance(0.01)   # Allow 1 cm position error
    move_group.set_goal_orientation_tolerance(0.05) # Allow slight wrist rotation error (radians)
    move_group.set_goal_joint_tolerance(0.05)      # Allow slight joint angle error (radians)
    # moveit_commander.

    rospy.loginfo("Executing MOVJ to home/ready position...")
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = math.radians(0)
    joint_goal[1] = math.radians(10)
    joint_goal[2] = math.radians(0)
    joint_goal[3] = math.radians(0)
    joint_goal[4] = math.radians(0)
    joint_goal[5] = math.radians(0)

    move_group.go(joint_goal, wait=True)
    move_group.stop()
    rospy.sleep(1.0)

    rospy.loginfo("Executing sequential point-to-point movement...")

    # 1. CAPTURE THE PERFECT ORIENTATION FROM MOVJ
    # We will use this stable wrist angle for all subsequent points
    target_pose = move_group.get_current_pose().pose

    # Define the list of absolute waypoints (X, Y, Z)
    waypoints = [
        [0.25, 0.10, 0.35], # Point 1: Hover above
        [0.25, 0.10, 0.3], # Point 2: Move down to pick
        [0.25, -0.10, 0.3], # Point 3: Move laterally
        [0.25, -0.10, 0.35]  # Point 4: Move up
    ]

    # Draw the path in RViz before moving
    draw_path_marker(waypoints, marker_pub)

    rospy.loginfo("Executing sequential point-to-point movement...")

    # Iterate through each waypoint
    for i, point in enumerate(waypoints):
        rospy.loginfo(f"Moving to Point {i+1}: X={point[0]}, Y={point[1]}, Z={point[2]}")
        
        move_group.set_start_state_to_current_state()

        # 2. UPDATE ONLY THE POSITION, KEEP THE CAPTURED ORIENTATION
        target_pose.position.x = point[0]
        target_pose.position.y = point[1]
        target_pose.position.z = point[2]
        
        # 3. USE SET_POSE_TARGET TO ENFORCE BOTH POSITION AND ORIENTATION
        move_group.set_pose_target(target_pose)
        
        success = move_group.go(wait=True)
        
        move_group.stop()
        move_group.clear_pose_targets()
        
        
        if not success:
            rospy.logwarn(f"Failed to reach Point {i+1}. Collision detected or limit reached.")
            break
            
        rospy.sleep(2)

    rospy.loginfo("Sequence finished.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass