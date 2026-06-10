#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

struct StampConfig {
    double tilt_deg;
    double azimuth_deg;
    std::string name;
};

// 1. Generate the Zig-Zag Grid on the silicone workpiece
std::vector<tf2::Vector3> generateZigZagGrid() {
    std::vector<tf2::Vector3> surface_points;
    double width = 0.016;  
    double length = 0.024; 
    double target_z = 0.005; 
    double center_offset[3] = {0.2, 0.0, 0.2}; 
    double step_size = 0.008; 
    bool left_to_right = true;

    for (double y_local = -width / 2; y_local <= width / 2 + 1e-5; y_local += step_size) {
        std::vector<double> row_x_points;
        for (double x_local = -length / 2; x_local <= length / 2 + 1e-5; x_local += step_size) {
            row_x_points.push_back(x_local);
        }
        if (!left_to_right) std::reverse(row_x_points.begin(), row_x_points.end());
        left_to_right = !left_to_right;

        for (double x_local : row_x_points) {
            double world_x = center_offset[0] + x_local;
            double world_y = center_offset[1] + y_local;
            double world_z = center_offset[2] + target_z;
            surface_points.push_back(tf2::Vector3(world_x, world_y, world_z));
        }
    }
    return surface_points;
}

geometry_msgs::Pose computeTCPPose(const tf2::Vector3& target_point, double tilt_deg, double azimuth_deg, double offset_dist) {
    tf2::Vector3 normal_vector(0.0, 0.0, 1.0);
    tf2::Vector3 z_axis_tool = -normal_vector;
    tf2::Vector3 x_ref(1.0, 0.0, 0.0);
    if (std::abs(z_axis_tool.dot(x_ref)) > 0.99) x_ref = tf2::Vector3(0.0, 1.0, 0.0);

    tf2::Vector3 y_axis_tool = z_axis_tool.cross(x_ref).normalized();
    tf2::Vector3 x_axis_tool = y_axis_tool.cross(z_axis_tool).normalized();

    tf2::Matrix3x3 rotation_matrix(
        x_axis_tool.x(), y_axis_tool.x(), z_axis_tool.x(),
        x_axis_tool.y(), y_axis_tool.y(), z_axis_tool.y(),
        x_axis_tool.z(), y_axis_tool.z(), z_axis_tool.z()
    );
    tf2::Quaternion q_base;
    rotation_matrix.getRotation(q_base);

    tf2::Quaternion q_urdf_compensation;
    q_urdf_compensation.setRPY(M_PI, 0.0, 0.0);

    double tilt_rad = tilt_deg * M_PI / 180.0;
    double azimuth_rad = azimuth_deg * M_PI / 180.0;
    
    tf2::Quaternion q_yaw, q_tilt;
    q_yaw.setRPY(0.0, 0.0, azimuth_rad);
    q_tilt.setRPY(tilt_rad, 0.0, 0.0);

    tf2::Quaternion q_final = q_base * q_yaw * q_tilt * q_urdf_compensation;
    q_final.normalize();

    tf2::Matrix3x3 final_matrix(q_final);
    tf2::Vector3 actual_z_dir = final_matrix.getColumn(2);

    tf2::Vector3 tcp_position = target_point + (actual_z_dir * offset_dist);

    geometry_msgs::Pose pose;
    pose.orientation.x = q_final.x();
    pose.orientation.y = q_final.y();
    pose.orientation.z = q_final.z();
    pose.orientation.w = q_final.w();
    pose.position.x = tcp_position.x();
    pose.position.y = tcp_position.y();
    pose.position.z = tcp_position.z();

    return pose;
}

// 3. FUSION WATCHDOG PTP: Joint Space Evaluation
bool executeStablePTP(moveit::planning_interface::MoveGroupInterface& move_group, 
                      const geometry_msgs::Pose& target_pose, 
                      const std::string& info_text) {
    
    move_group.setGoalPositionTolerance(0.005);
    move_group.setGoalOrientationTolerance(0.1);

    sensor_msgs::JointStateConstPtr current_joints_msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(1.5));
    if (!current_joints_msg) {
        ROS_FATAL("CONNECTION LOST at %s!", info_text.c_str());
        return false; 
    }

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    current_state->setVariablePositions(current_joints_msg->name, current_joints_msg->position);
    move_group.setStartState(*current_state);
    
    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Moving to: %s", info_text.c_str());
        
        double expected_duration = plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec();
        double timeout = expected_duration + 5.0; 
        std::vector<double> target_joints = plan.trajectory_.joint_trajectory.points.back().positions;

        move_group.asyncExecute(plan); 

        ros::Rate rate(10); 
        ros::Time start_time = ros::Time::now();
        bool reached_target = false;
        
        std::vector<double> last_joints = move_group.getCurrentJointValues();
        int stable_count = 0;

        while (ros::ok() && (ros::Time::now() - start_time).toSec() < timeout) {
            double elapsed = (ros::Time::now() - start_time).toSec();
            std::vector<double> curr_joints = move_group.getCurrentJointValues();
            
            double max_joint_error = 1.0; // Default high

            if (curr_joints.size() == last_joints.size() && curr_joints.size() == target_joints.size()) {
                double max_movement = 0.0;
                max_joint_error = 0.0;

                for (size_t i = 0; i < curr_joints.size(); ++i) {
                    // Check movement speed (stability)
                    double movement = std::abs(curr_joints[i] - last_joints[i]);
                    if (movement > max_movement) max_movement = movement;

                    // Check mathematical distance to target
                    double error = std::abs(curr_joints[i] - target_joints[i]);
                    if (error > max_joint_error) max_joint_error = error;
                }

                if (max_movement < 0.0005) {
                    stable_count++;
                } else {
                    stable_count = 0;
                }
            }
            last_joints = curr_joints;

            // FUSION SUCCESS: Elapsed at least 0.2s AND within 4.5 degrees (0.08 rad) AND physically stopped
            if (elapsed > 0.2 && max_joint_error < 0.1 && stable_count >= 3) {
                reached_target = true;
                break;
            }

            rate.sleep();
        }

        move_group.stop(); 
        ros::Duration(0.5).sleep(); 

        if (!reached_target) ROS_WARN("Watchdog Timeout: Failed to reach %s", info_text.c_str());
        return reached_target;
    }
    
    ROS_ERROR("Planning failed for: %s.", info_text.c_str());
    return false;
}

// 4. FUSION CARTESIAN STAMP: Masked Time Evaluation
bool executeCartesianStamp(moveit::planning_interface::MoveGroupInterface& move_group, 
                           const geometry_msgs::Pose& theoretical_press_pose, 
                           const std::string& info_text) {
    
    sensor_msgs::JointStateConstPtr current_joints_msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(1.5));
    if (!current_joints_msg) {
        ROS_FATAL("CONNECTION LOST at %s!", info_text.c_str());
        return false;
    }

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    current_state->setVariablePositions(current_joints_msg->name, current_joints_msg->position);
    move_group.setStartState(*current_state);

    const Eigen::Isometry3d& end_effector_state = current_state->getGlobalLinkTransform("robot_tcp");
    Eigen::Quaterniond q(end_effector_state.rotation());
    
    geometry_msgs::Pose actual_start_pose;
    actual_start_pose.position.x = end_effector_state.translation().x();
    actual_start_pose.position.y = end_effector_state.translation().y();
    actual_start_pose.position.z = end_effector_state.translation().z();
    actual_start_pose.orientation.x = q.x();
    actual_start_pose.orientation.y = q.y();
    actual_start_pose.orientation.z = q.z();
    actual_start_pose.orientation.w = q.w();

    geometry_msgs::Pose actual_press_pose = theoretical_press_pose;
    actual_press_pose.orientation = actual_start_pose.orientation;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(actual_press_pose);  
    waypoints.push_back(actual_start_pose);  

    moveit_msgs::RobotTrajectory trajectory;
    const double eef_step = 0.005; 
    const double jump_threshold = 0.0;
    
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory, jump_threshold);

    if (fraction >= 0.99) {
        robot_trajectory::RobotTrajectory rt(current_state->getRobotModel(), "manipulator");
        rt.setRobotTrajectoryMsg(*current_state, trajectory);
        
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        if (iptp.computeTimeStamps(rt, 0.4, 0.4)) {
            rt.getRobotTrajectoryMsg(trajectory);
            
            size_t last_idx = trajectory.joint_trajectory.points.size() - 1;
            trajectory.joint_trajectory.points[last_idx].velocities.assign(6, 0.0);
            trajectory.joint_trajectory.points[last_idx].accelerations.assign(6, 0.0);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            
            ROS_INFO("Executing Cartesian Stamp: %s", info_text.c_str());

            double expected_duration = trajectory.joint_trajectory.points.back().time_from_start.toSec();
            double timeout = expected_duration + 3.0;
            std::vector<double> target_joints = trajectory.joint_trajectory.points.back().positions;
            
            move_group.asyncExecute(plan);

            ros::Rate rate(10);
            ros::Time start_time = ros::Time::now();
            bool reached_target = false;
            
            std::vector<double> last_joints = move_group.getCurrentJointValues();
            int stable_count = 0;

            while (ros::ok() && (ros::Time::now() - start_time).toSec() < timeout) {
                double elapsed = (ros::Time::now() - start_time).toSec();
                std::vector<double> curr_joints = move_group.getCurrentJointValues();
                
                double max_joint_error = 1.0;

                if (curr_joints.size() == last_joints.size() && curr_joints.size() == target_joints.size()) {
                    double max_movement = 0.0;
                    max_joint_error = 0.0;

                    for (size_t i = 0; i < curr_joints.size(); ++i) {
                        double movement = std::abs(curr_joints[i] - last_joints[i]);
                        if (movement > max_movement) max_movement = movement;

                        double error = std::abs(curr_joints[i] - target_joints[i]);
                        if (error > max_joint_error) max_joint_error = error;
                    }

                    if (max_movement < 0.0005) {
                        stable_count++;
                    } else {
                        stable_count = 0;
                    }
                }
                last_joints = curr_joints;

                // TIME MASKING: Watchdog chỉ được phép kiểm tra sau khi 75% thời gian quỹ đạo đã trôi qua
                // Đảm bảo kim đã đâm xuống sâu nhất và đang trong quá trình rút lên
                if (elapsed > (expected_duration * 0.75) && max_joint_error < 0.08 && stable_count >= 3) {
                    reached_target = true;
                    break;
                }
                
                rate.sleep();
            }

            move_group.stop();
            return true;
        }
    }
    
    ROS_WARN("Failed to compute Cartesian path for: %s", info_text.c_str());
    return false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "safe_combined_stamping");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group.setEndEffectorLink("robot_tcp");
    move_group.setPoseReferenceFrame("world");

    move_group.setMaxVelocityScalingFactor(0.4);
    move_group.setMaxAccelerationScalingFactor(0.4);

    std::vector<tf2::Vector3> target_grid = generateZigZagGrid();
    ROS_INFO("Generated %lu surface points on the silicone.", target_grid.size());

    std::vector<StampConfig> configs = {
        {0.0,   0.0,   "STRAIGHT"},
        {15.0,  0.0,   "TILT_EAST"},
        {15.0,  45.0,  "TILT_EAST_2"},
        {15.0,  90.0,  "TILT_NORTH"},
        {15.0,  135.0, "TILT_NORTH_2"},
        {15.0,  180.0, "TILT_WEST"},
        {15.0,  225.0, "TILT_WEST_2"}, 
        {15.0,  270.0, "TILT_SOUTH"},
        {15.0,  315.0, "TILT_SOUTH_2"} 
    };

    double hover_distance = 0.003;  
    double press_distance = -0.003;  

    unsigned int cycle_counter = 1;

    while (ros::ok()) {
        ROS_INFO("==================================================");
        ROS_INFO("STARTING CYCLE #%u", cycle_counter);
        ROS_INFO("==================================================");

        for (size_t p = 0; p < target_grid.size(); ++p) {
            if (!ros::ok()) break;
            ROS_INFO("--- Point [%zu / %zu] of Cycle #%u ---", p + 1, target_grid.size(), cycle_counter);
            
            for (const auto& config : configs) {
                if (!ros::ok()) break;

                geometry_msgs::Pose hover_pose = computeTCPPose(target_grid[p], config.tilt_deg, config.azimuth_deg, hover_distance);
                geometry_msgs::Pose press_pose = computeTCPPose(target_grid[p], config.tilt_deg, config.azimuth_deg, press_distance);

                std::string prefix = "Cycle_" + std::to_string(cycle_counter) + "_Pt_" + std::to_string(p + 1) + "_" + config.name;

                if (executeStablePTP(move_group, hover_pose, prefix + " [HOVER]")) {
                    ros::Duration(3.0).sleep(); 
                    
                    executeCartesianStamp(move_group, press_pose, prefix + " [STAMP]");
                    ros::Duration(3.0).sleep(); 
                } else {
                    ROS_WARN("Skipping STAMP due to HOVER failure at: %s", prefix.c_str());
                    continue; 
                }
            }
        }

        if (!ros::ok()) break;

        ROS_INFO("Finished Cycle #%u. Performing safe retreat...", cycle_counter);
        geometry_msgs::Pose safe_lift_pose = move_group.getCurrentPose().pose;
        safe_lift_pose.position.z += 0.020; 
        executeStablePTP(move_group, safe_lift_pose, "SAFE_LIFT_UP");
        ros::Duration(3.0).sleep();
        
        cycle_counter++;
    }

    ROS_INFO("Task complete. Shutting down.");
    ros::shutdown();
    return 0;
}