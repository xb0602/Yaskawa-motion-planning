#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <vector>
#include <algorithm>

void drawPathMarker(const std::vector<geometry_msgs::Pose>& waypoints, ros::Publisher& publisher){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "robot_path";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1.0; 
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;

    marker.scale.x = 0.005;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    for (const auto &wp : waypoints){       //for (int i = 0; i < waypoints.size(); i++)
        geometry_msgs::Point p;
        p.x = wp.position.x;
        p.y = wp.position.y;
        p.z = wp.position.z;
        marker.points.push_back(p);
    }

    // Chờ đợi RViz kết nối thành công trước khi bắn tin nhắn ---
    ros::Rate poll_rate(10);
    int attempts = 0;
    while (publisher.getNumSubscribers() == 0 && ros::ok() && attempts < 20) {
        ROS_INFO_THROTTLE(1, "Waiting for RViz to subscribe to the marker topic...");
        poll_rate.sleep();
        attempts++;
    }

    publisher.publish(marker);
    ROS_INFO("Path Marker published.");
}

std::vector<geometry_msgs::Pose> generateStampingWaypoints()
{
    std::vector<geometry_msgs::Pose> waypoints;

    // --- 1. Thông số hình học ---
    double sphere_radius = 0.02;    
    double center_offset[3] = {0.27, 0.0, 0.25}; 
    double tool_length = 0.005;  // Khoảng bù công cụ

    // --- 2. Thông số dập lực (Stamping) ---
    double hover_dist = 0.003; // Chờ cách bề mặt 3mm (0.003m)
    double press_dist = 0.003; // Đâm lún sâu vào 5mm (0.005m)
    
    // Tăng step_size lên 1cm (0.01m) để thưa ra. Nếu để 5mm robot dập sẽ rất lâu.
    double step_size = 0.001;   
    bool left_to_right = true;

    double safe_factor = 0.5; // Giới hạn % cho cả X và Y
    double safe_limit = safe_factor * sphere_radius;

    // Quét lưới dọc trục Y
    for (double y_local = - safe_limit; y_local <= safe_limit + 1e-5; y_local += step_size) {
        
        double y_ratio = y_local / sphere_radius;
        if (y_ratio > 1.0) y_ratio = 1.0;
        if (y_ratio < -1.0) y_ratio = -1.0;

        double max_spherical_x = std::sqrt(std::pow(sphere_radius,2) - std::pow(y_local,2));

        double x_start = std::max(-max_spherical_x, -safe_limit);
        double x_end = std::min(max_spherical_x, safe_limit);

        std::vector<double> row_x_points;
        for (double x = x_start; x <= x_end + 1e-5; x += step_size) {
            row_x_points.push_back(x);
        }

        if (!left_to_right) {
            std::reverse(row_x_points.begin(), row_x_points.end());
        }
        left_to_right = !left_to_right;

        // Xử lý từng điểm trên bề mặt
        for (double x_local : row_x_points) {
            
            // Tính độ cao Z trên mặt vòm
            double target_domain = std::pow(sphere_radius, 2) - std::pow(x_local, 2) - std::pow(y_local, 2);
            if (target_domain < 0.0) target_domain = 0.0;
            double z_local = std::sqrt(target_domain);

            // Tọa độ gốc ngay trên bề mặt phôi (chưa tính bù dao)
            double base_x = center_offset[0] + x_local;
            double base_y = center_offset[1] + y_local;
            double base_z = center_offset[2] + z_local;

            // Tính pháp tuyến
            tf2::Vector3 normal_vector(x_local, y_local, z_local);
            if (normal_vector.length() > 0.0001) normal_vector.normalize();     //nx = x_local / sphere_radius; ny = y_local / sphere_radius; nz = z_local / sphere_radius;
            else normal_vector = tf2::Vector3(0, 0, 1); 

            // Tính Toán góc xoay vuông góc (Quaternion)
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

            tf2::Quaternion quat;
            rotation_matrix.getRotation(quat);

            // --- 3. TẠO COMBO 3 ĐIỂM STAMPING ---
            geometry_msgs::Pose hover_pose, press_pose;
            
            // Góc xoay giữ nguyên y hệt nhau để robot đâm thẳng tắp
            hover_pose.orientation.x = quat.x();
            hover_pose.orientation.y = quat.y();
            hover_pose.orientation.z = quat.z();
            hover_pose.orientation.w = quat.w();
            press_pose.orientation = hover_pose.orientation;

            // A. Tọa độ điểm Hover (Lùi ra ngoài không khí)
            hover_pose.position.x = base_x + normal_vector.x() * (tool_length + hover_dist);
            hover_pose.position.y = base_y + normal_vector.y() * (tool_length + hover_dist);
            hover_pose.position.z = base_z + normal_vector.z() * (tool_length + hover_dist);

            // B. Tọa độ điểm Press (Đâm lún vào vật liệu)
            press_pose.position.x = base_x + normal_vector.x() * (tool_length - press_dist);
            press_pose.position.y = base_y + normal_vector.y() * (tool_length - press_dist);
            press_pose.position.z = base_z + normal_vector.z() * (tool_length - press_dist);

            // Nạp vào mảng quỹ đạo theo thứ tự: Chờ -> Nhấn lún -> Rút lên lại chờ
            waypoints.push_back(hover_pose);
            waypoints.push_back(press_pose);
            waypoints.push_back(hover_pose);
        }
    }

    return waypoints;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "Oval_Tracking_CPP");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",10);

    const std::string PLANNING_GROUP = "manipulator";
    std::string robot_name = "motomini";
    ROS_INFO("Robot name is %s", robot_name.c_str());


    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPoseReferenceFrame("world");

    //Limit velocity and acceleration to 10%
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);

    //Error setting
    move_group.setGoalPositionTolerance(0.01);
    move_group.setGoalOrientationTolerance(0.05);
    move_group.setGoalJointTolerance(0.05);

    //Get data of current joint
    std::vector<double> joint_group_positions = move_group.getCurrentJointValues();

    joint_group_positions[0] = 0.0;
    joint_group_positions[1] = 0.0;
    joint_group_positions[2] = 0.0;
    joint_group_positions[3] = 0.0;
    joint_group_positions[4] = 0.0;
    joint_group_positions[5] = 0.0;

    move_group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success_joint = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success_joint){
        move_group.execute(my_plan);
        ROS_INFO("Move to initial position");
    }
    ros::Duration(1.0).sleep();

    move_group.setStartStateToCurrentState();

    std::vector<geometry_msgs::Pose> generated_trajectory = generateStampingWaypoints();
    ROS_INFO("Successfully generated %lu waypoints with automatic normal orientation using C++!", 
             generated_trajectory.size());

    
    ROS_INFO("Moving to the STARTING POINT of the oval safely...");
    move_group.setPoseTarget(generated_trajectory[0]);
    
    moveit::planning_interface::MoveGroupInterface::Plan start_pt_plan;
    if (move_group.plan(start_pt_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        move_group.execute(start_pt_plan);
        ROS_INFO("Arrived at the starting point.");
    } else {
        ROS_ERROR("Cannot reach the starting point. Check your tool_length or workspace limits!");
        ros::shutdown();
        return -1;
    }

    // Đợi robot dừng hẳn và cập nhật lại trạng thái gốc
    ros::Duration(0.5).sleep();

    while (ros::ok())
    {
        drawPathMarker(generated_trajectory, marker_pub);
        move_group.setStartStateToCurrentState();

        moveit_msgs::RobotTrajectory trajectory;
        const double eef_step = 0.001;
        moveit_msgs::MoveItErrorCodes error_code;

        double fraction = move_group.computeCartesianPath(generated_trajectory, eef_step, trajectory, true, &error_code);

        if (fraction > 0.9){
            ROS_INFO("Path planned successfully (Fraction: %f). Executing...", fraction);

            // Tạo biến Plan rỗng và nhét quỹ đạo vừa tính được vào
            moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
            cartesian_plan.trajectory_ = trajectory;
            
            // CHỈ GỌI LỆNH NÀY DUY NHẤT 1 LẦN
            moveit::core::MoveItErrorCode result = move_group.execute(cartesian_plan);
            
            if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                ROS_INFO("Execution finished perfectly! Exiting node...");
            }
            else {
                ROS_ERROR("Execution failed with error code: %d", result.val);
            }
        }

        else {
            ROS_WARN("Could not compute the full path. Collision or singularity detected. Fraction: %f", fraction);
        }

        ros::Duration(0.1).sleep();
    }
    
    ROS_INFO("Stopping robot controller gracefully...");
    spinner.stop();
    ros::shutdown();
    return 0;
    
    
}