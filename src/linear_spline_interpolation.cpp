#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <vector>

void drawPathMarker(const std::vector<geometry_msgs::Pose>& waypoints, ros::Publisher& publisher){
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
        p.x = wp.position.x;
        p.y = wp.position.y;
        p.z = wp.position.z;
        marker.points.push_back(p);
    }

    publisher.publish(marker);
    ROS_INFO("Path Marker published.");
}


int main(int argc, char** argv){
    ros::init(argc,argv, "linear_spline_interpolation_cpp");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",10);

    const std::string PLANNING_GROUP = "manipulator";
    std::string robot_name = "MotoMINI";
    ROS_INFO("Robot name is: %s", robot_name.c_str());

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

    geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
    //kiểu dữ liệu đầu vào của computeCartesianPath là std::vector<geometry_msgs::Pose>
    std::vector<geometry_msgs::Pose> waypoints;

    double radius = 0.02;
    double center_x = target_pose.position.x;
    double center_y = target_pose.position.y + radius;
    int resolution = 20;

    for (int i = 0; i <= resolution; ++i){
        double angle = (M_PI/resolution)*i;

        target_pose.position.x = center_x + radius*cos(angle - M_PI_2);
        target_pose.position.y = center_y + radius*sin(angle - M_PI_2);

        waypoints.push_back(target_pose);
    }

    drawPathMarker(waypoints, marker_pub);

    //Quy hoạch quỹ đạo nội suy (Cartesian Path)
    moveit_msgs::RobotTrajectory trajectory;
    const double eef_step = 0.01;           // Bước nội suy 1 cm
    const double jump_threshold = 0.0;      // Tương đương tham số True trong Python để tránh bước nhảy lạ

    ROS_INFO("Calculating circular path...");
    moveit_msgs::MoveItErrorCodes error_code;
    double fraction =  move_group.computeCartesianPath(waypoints, eef_step, trajectory, true, &error_code);
    
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

    spinner.stop();
    ros::shutdown();
    return 0;
    
}