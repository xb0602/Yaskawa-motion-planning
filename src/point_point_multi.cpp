#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

// Structure to define multi-angle stamping configurations
struct StampConfig {
    double tilt_deg;
    double azimuth_deg;
    std::string name;
};

// Function 1: Generate purely mathematical target points in a Zig-Zag pattern
std::vector<tf2::Vector3> generateZigZagGrid() {
    std::vector<tf2::Vector3> surface_points;

    // Geometric parameters for the rectangular surface
    double width = 0.016;  // Y-axis width
    double length = 0.024;  // X-axis length
    double target_z = 0.005; // Z-axis height of the surface
    double center_offset[3] = {0.20, 0.0, 0.25}; // Center anchor of the workpiece

    double step_size = 0.008; // Step size for scanning
    bool left_to_right = true;

    // Scan along Y axis
    for (double y_local = -width / 2; y_local <= width / 2 + 1e-5; y_local += step_size) {
        
        std::vector<double> row_x_points;
        for (double x_local = -length / 2; x_local <= length / 2 + 1e-5; x_local += step_size) {
            row_x_points.push_back(x_local);
        }

        // Reverse row to create Zig-Zag motion
        if (!left_to_right) {
            std::reverse(row_x_points.begin(), row_x_points.end());
        }
        left_to_right = !left_to_right;

        // Apply offset to create real world coordinates
        for (double x_local : row_x_points) {
            double world_x = center_offset[0] + x_local;
            double world_y = center_offset[1] + y_local;
            double world_z = center_offset[2] + target_z;
            
            surface_points.push_back(tf2::Vector3(world_x, world_y, world_z));
        }
    }
    
    return surface_points;
}

// Function 2: Compute exact tool pose for a given target point and orientation
geometry_msgs::Pose computeTCPPose(const tf2::Vector3& target_point, double tilt_deg, double azimuth_deg, double offset_dist) {
    // Surface normal (Flat surface pointing up)
    tf2::Vector3 normal_vector(0.0, 0.0, 1.0);
    tf2::Vector3 z_axis_tool = -normal_vector;
    tf2::Vector3 x_ref(1.0, 0.0, 0.0);
    if (std::abs(z_axis_tool.dot(x_ref)) > 0.99) {
        x_ref = tf2::Vector3(0.0, 1.0, 0.0);
    }

    tf2::Vector3 y_axis_tool = z_axis_tool.cross(x_ref).normalized();
    tf2::Vector3 x_axis_tool = y_axis_tool.cross(z_axis_tool).normalized();

    tf2::Matrix3x3 rotation_matrix(
        x_axis_tool.x(), y_axis_tool.x(), z_axis_tool.x(),
        x_axis_tool.y(), y_axis_tool.y(), z_axis_tool.y(),
        x_axis_tool.z(), y_axis_tool.z(), z_axis_tool.z()
    );
    tf2::Quaternion q_base;
    rotation_matrix.getRotation(q_base);

    // Hardware compensation for Motomini flange orientation
    tf2::Quaternion q_urdf_compensation;
    q_urdf_compensation.setRPY(M_PI, 0.0, 0.0);

    // Apply Tilt and Azimuth rotations
    double tilt_rad = tilt_deg * M_PI / 180.0;
    double azimuth_rad = azimuth_deg * M_PI / 180.0;
    
    tf2::Quaternion q_yaw, q_tilt;
    q_yaw.setRPY(0.0, 0.0, azimuth_rad);
    q_tilt.setRPY(tilt_rad, 0.0, 0.0);

    tf2::Quaternion q_final = q_base * q_yaw * q_tilt * q_urdf_compensation;
    q_final.normalize();

    // Extract actual Z direction to retreat along the tool axis
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

// Function 3: Wrapper to safely execute Point-To-Point move and clear memory
bool executePTPMove(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Pose& target_pose, const std::string& info_text) {
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (success) {
        ROS_INFO("Moving to: %s", info_text.c_str());
        moveit::core::MoveItErrorCode result = move_group.execute(plan);
        return (result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    
    ROS_ERROR("Planning failed for: %s", info_text.c_str());
    return false;
}

// Function 4: Publish visualization markers to RViz to verify trajectory beforehand
void publishGridMarkers(const std::vector<tf2::Vector3>& points, ros::Publisher& marker_pub) {
    // A. Line Strip Marker for Zig-Zag trajectory path
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "zigzag_path";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    
    line_strip.scale.x = 0.001; // Line width 1mm
    line_strip.color.r = 1.0;   // Red path
    line_strip.color.a = 1.0;

    // B. Points Marker for discrete stamping targets
    visualization_msgs::Marker points_marker;
    points_marker.header.frame_id = "world";
    points_marker.header.stamp = ros::Time::now();
    points_marker.ns = "stamping_points";
    points_marker.action = visualization_msgs::Marker::ADD;
    points_marker.pose.orientation.w = 1.0;
    points_marker.id = 1;
    points_marker.type = visualization_msgs::Marker::POINTS;

    points_marker.scale.x = 0.003; // Point size 3mm
    points_marker.scale.y = 0.003;
    points_marker.color.g = 1.0;   // Green points
    points_marker.color.a = 1.0;

    for (const auto& pt : points) {
        geometry_msgs::Point p;
        p.x = pt.x();
        p.y = pt.y();
        p.z = pt.z();

        line_strip.points.push_back(p);
        points_marker.points.push_back(p);
    }

    ros::Rate poll_rate(10);
    int attempts = 0;
    while (marker_pub.getNumSubscribers() == 0 && ros::ok() && attempts < 15) {
        ROS_INFO_THROTTLE(1, "Waiting for RViz to subscribe to the marker topic...");
        poll_rate.sleep();
        attempts++;
    }

    marker_pub.publish(line_strip);
    marker_pub.publish(points_marker);
    ROS_INFO("Visualization markers successfully broadcasted to RViz.");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "PTP_ZigZag_Infinite_Stamping");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Publisher for RViz visualization
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPoseReferenceFrame("world");
    
    // IMPORTANT: Using the strictly corrected URDF TCP
    move_group.setEndEffectorLink("robot_tcp");

    // Reduce speed slightly for hardware safety during repeated PTP moves
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);

    move_group.setGoalPositionTolerance(0.005);
    move_group.setGoalOrientationTolerance(0.05);

    // 1. Generate the barebone Zig-Zag coordinate list
    std::vector<tf2::Vector3> target_grid = generateZigZagGrid();
    ROS_INFO("Successfully generated %lu surface points in Zig-Zag pattern.", target_grid.size());

    // 2. Define angles
    std::vector<StampConfig> configs = {
        {0.0,   0.0,   "STRAIGHT"},
        {15.0,  0.0,   "TILT_EAST"},
        {15.0,  90.0,  "TILT_NORTH"},
        {15.0,  180.0, "TILT_WEST"},
        {15.0,  270.0, "TILT_SOUTH"}
    };

    double hover_distance = 0.003; // Hover 3mm above
    double press_distance = -0.003;  // Touch the surface exactly

    unsigned int cycle_counter = 1;

    // --- 3. INFINITE EXECUTION LOOP ---
    while (ros::ok()) {
        ROS_INFO("==================================================");
        ROS_INFO("STARTING SURFACE SCANNING CYCLE #%u", cycle_counter);
        ROS_INFO("==================================================");

        // Always refresh markers at the beginning of each cycle to keep them visible in RViz
        publishGridMarkers(target_grid, marker_pub);

        for (size_t p = 0; p < target_grid.size(); ++p) {
            
            // Check if ROS is still active inside the nested loop to allow clean Ctrl+C exits
            if (!ros::ok()) break;

            ROS_INFO("--- Point [%zu / %zu] of Cycle #%u ---", p + 1, target_grid.size(), cycle_counter);
            
            for (const auto& config : configs) {
                if (!ros::ok()) break;

                // Generate exact poses for this specific angle on the fly
                geometry_msgs::Pose hover_pose = computeTCPPose(target_grid[p], config.tilt_deg, config.azimuth_deg, hover_distance);
                geometry_msgs::Pose press_pose = computeTCPPose(target_grid[p], config.tilt_deg, config.azimuth_deg, press_distance);

                std::string prefix = "Cycle_" + std::to_string(cycle_counter) + "_Pt_" + std::to_string(p + 1) + "_" + config.name;

                // Step A: Hover 1
                if (!executePTPMove(move_group, hover_pose, prefix + " [HOVER]")) {
                    ROS_WARN("Skipping angle due to IK/Planning failure.");
                    continue;
                }
                ros::Duration(3.0).sleep(); 

                // Step B: Press
                if (executePTPMove(move_group, press_pose, prefix + " [PRESS]")) {
                    ros::Duration(3.0).sleep(); 
                    
                    // Step C: Hover 2 (Pull back)
                    executePTPMove(move_group, hover_pose, prefix + " [RETRACT]");
                }
                ros::Duration(3.0).sleep(); 
            }
        }

        if (!ros::ok()) break;

        // --- SAFE RETRACT LOGIC BEFORE RESETTING ---
        ROS_INFO("Finished all points in Cycle #%u. Performing safe retreat...", cycle_counter);
        
        // Lift the tool straight up by 15mm from its last position to clear obstacles
        geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
        geometry_msgs::Pose safe_lift_pose = current_pose;
        safe_lift_pose.position.z += 0.015; 
        
        executePTPMove(move_group, safe_lift_pose, "SAFE_LIFT_UP");
        ros::Duration(5.0).sleep();

        cycle_counter++;
    }

    ROS_INFO("Infinite execution loop terminated. Stopping robot controller gracefully...");
    spinner.stop();
    ros::shutdown();
    return 0;
}