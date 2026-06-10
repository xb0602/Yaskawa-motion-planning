#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <vector>

void drawPathMarker(const std::vector<std::vector<double>>& waypoints, ros::Publisher& publisher){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "robot_path";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.005;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    for (const auto &wp : waypoints){       //for (int i = 0; i < waypoints.size(); i++)
        geometry_msgs::Point p;
        p.x = wp[0];
        p.y = wp[1];
        p.z = wp[2];
        marker.points.push_back(p);
    }

    publisher.publish(marker);
    ROS_INFO("Path Marker published.");
}


int main(int argc, char** argv){
    ros::init(argc,argv, "Point_to_point_CPP");
    ros::NodeHandle nh;

    //chạy một Spinner ngầm để cập nhật trạng thái robot liên tục
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Thiết lập Publisher để vẽ Marker lên RViz
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",10);

    const std::string PLANNING_GROUP = "manipulator";
    std::string robot_name = "MotoMINI";
    ROS_INFO("Robot name is: %s", robot_name.c_str());      //Hàm ROS_INFO thừa kế từ ngôn ngữ C, nên nó chỉ hiểu chuỗi ký tự kiểu C (char*). Nếu biến của bạn là kiểu chuỗi hiện đại của C++ (std::string), bạn bắt buộc phải dùng thêm hàm .c_str() để chuyển đổi,

    // Khởi tạo đối tượng MoveGroup trong C++ (Tương đương MoveGroupCommander trong Python)
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPoseReferenceFrame("world");

    //Limit velocity and acceleration to 10%
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);

    //Error setting
    move_group.setGoalPositionTolerance(0.01);
    move_group.setGoalOrientationTolerance(0.05);
    move_group.setGoalJointTolerance(0.05);

    //Get data of current joint
    std::vector<double> joint_group_positions = move_group.getCurrentJointValues();

    joint_group_positions[0] = 0.0;
    joint_group_positions[1] = (10.0 * M_PI/ 180.0);
    joint_group_positions[2] = 0.0;
    joint_group_positions[3] = 0.0;
    joint_group_positions[4] = 0.0;
    joint_group_positions[5] = 0.0;

    move_group.setJointValueTarget(joint_group_positions);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success_joint = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success_joint){
        move_group.execute(my_plan);
    }
    ros::Duration(1.0).sleep();

    ROS_INFO("Executing sequential point-to-point movement...");

    geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;

    std::vector<std::vector<double>> waypoints = {
        {0.25,  0.10, 0.30},  // Điểm 1: Vị trí chờ phía trên
        {0.25,  0.10, 0.25},  // Điểm 2: Hạ thấp xuống để gắp
        {0.25, -0.10, 0.25},  // Điểm 3: Di chuyển ngang sang trạm phân loại
        {0.25, -0.10, 0.30}   // Điểm 4: Nhấc lên sau khi nhả vật
    };

    drawPathMarker(waypoints, marker_pub);
    
    for (size_t i = 0; i < waypoints.size(); ++i){
        ROS_INFO("Moving to Point %zu: X=%.2f, Y=%.2f, Z=%.2f", i + 1, waypoints[i][0], waypoints[i][1], waypoints[i][2]);

        move_group.setStartStateToCurrentState();

        // Cập nhật tọa độ tịnh tiến X, Y, Z nhưng giữ nguyên góc xoay Quaternion của cổ tay
        target_pose.position.x = waypoints[i][0];
        target_pose.position.y = waypoints[i][1];
        target_pose.position.z = waypoints[i][2];

        // Gán mục tiêu và ra lệnh quy hoạch đường đi
        move_group.setPoseTarget(target_pose);
        
        bool success_cartesian = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (!success_cartesian) {
            ROS_WARN("Failed to reach Point %zu. Physical limit reached or collision detected!", i + 1);
            break; // Ngắt luồng an toàn nếu có một điểm bị lỗi vật lý
        }

        ros::Duration(0.5).sleep(); // Dừng nửa giây tại mỗi điểm để thiết bị ổn định động học
    }
    
    ROS_INFO("Sequence finished.");
    ros::shutdown();
    return 0;

}