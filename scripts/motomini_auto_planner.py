#!/usr/bin/env python3
import sys
import rospy
import copy
import math
import moveit_commander
from moveit_commander.move_group import MoveGroupCommander


def main():
    # Initialize the moveit_commander and the ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('motomini_auto_planner', anonymous=True)

    group_name = "manipulator"
    move_group: MoveGroupCommander = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_pose_reference_frame("world")

    move_group.set_planning_time(10.0)
    move_group.set_num_planning_attempts(10)
    # Set scaling factors to keep the physical robot movement slow and safe
    move_group.set_max_velocity_scaling_factor(0.05)
    move_group.set_max_acceleration_scaling_factor(0.05)
    

    rospy.loginfo("1. Waiting for valid robot state...")
    joint_goal = None

    # Vòng lặp đợi cho đến khi lấy được trạng thái khớp thực tế từ robot
    while joint_goal is None and not rospy.is_shutdown():
        joint_goal = move_group.get_current_joint_values()
        rospy.sleep(0.1)

    rospy.loginfo("1. Moving to a safe READY posture using Joint values...")
    move_group.set_goal_joint_tolerance(0.01)
    # Move to a posture where the gripper points perfectly downward
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = math.radians(0)   # Base facing forward
    joint_goal[1] = math.radians(10)  # Lower arm leaning slightly forward
    joint_goal[2] = math.radians(0)   # Upper arm straight
    joint_goal[3] = math.radians(0)   # Wrist roll centered
    joint_goal[4] = math.radians(0)# Pitch wrist down towards the table
    joint_goal[5] = math.radians(0)   # Tool roll centered

    move_group.set_joint_value_target(joint_goal)
    plan_success = move_group.go(wait=True)
    move_group.stop()
    if not plan_success:
        rospy.logerr("Could not reach Ready Posture. Check E-Stop or Mode!")
        return
    
    rospy.sleep(1.0)

    rospy.loginfo("2. Reading current downward pose and moving in Cartesian space...")
    # Get the current pose so we can keep the exact same downward orientation
    current_pose = move_group.get_current_pose().pose
    pose_goal = copy.deepcopy(current_pose)

    # Modify only the positional XYZ values within the safe workspace of MotoMINI
    # Target: 20cm forward, 10cm left, 15cm high from the base
    pose_goal.position.x = 0.25
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.3

    rospy.loginfo("Targeting Position - X: %.2f, Y: %.2f, Z: %.2f", 
                  pose_goal.position.x, pose_goal.position.y, pose_goal.position.z)

    # Use set_pose_target instead of set_position_target to enforce orientation
    move_group.set_pose_target(pose_goal)
    
    # Plan and execute the trajectory
    success = move_group.go(wait=True)
    
    move_group.stop()
    move_group.clear_pose_targets()

    if success:
        rospy.loginfo("Trajectory execution completed successfully. Ready to pick!")
    else:
        rospy.logwarn("Failed to execute the trajectory. Check collision or reachability.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

    # rospy.loginfo("Reading current pose...")
    
    # # 1. GET THE CURRENT POSE OF THE END-EFFECTOR
    # # This automatically captures valid X, Y, Z and a valid Quaternion (x, y, z, w)
    # current_pose = move_group.get_current_pose().pose
    
    # rospy.loginfo("Calculating trajectory to the target pose...")

    # # 2. CREATE A NEW GOAL POSE BASED ON THE CURRENT ONE
    # pose_goal = current_pose
    # rospy.loginfo(current_pose)
    # # 3. MODIFY ONLY THE POSITION (e.g., move 10cm forward on the X-axis)
    # pose_goal.position.x += 0.1
    # pose_goal.position.y += 0.0   
    # pose_goal.position.z -= 0.3

    # # Pass the target pose to the move group
    # move_group.set_pose_target(pose_goal)
    
    # Plan and execute the trajectory

