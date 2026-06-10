#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
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

    marker.scale.x = 0.002;

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

    // double total_tool_length = 0.03; 

    // for (const auto &wp : waypoints){
    //     // 1. Chuyển đổi góc xoay Quaternion của điểm đó thành Ma trận xoay để tìm các trục
    //     tf2::Quaternion quat;
    //     tf2::fromMsg(wp.orientation, quat);
    //     tf2::Matrix3x3 rotation_matrix(quat);

    //     // 2. Trích xuất trục Z của tay máy robot tại thời điểm đó
    //     // (Cột thứ 3 của ma trận xoay chính là Vector hướng của trục Z công cụ)
    //     tf2::Vector3 z_axis_tool = rotation_matrix.getColumn(2);

    //     geometry_msgs::Point p;
    //     // 3. TOÁN HỌC THÔNG MINH: Lấy vị trí mặt bích, cộng tịnh tiến dọc theo trục Z của chính nó 
    //     // một khoảng bằng chiều dài công cụ để ra được vị trí chóp kim thật!
    //     p.x = wp.position.x - z_axis_tool.x() * total_tool_length;
    //     p.y = wp.position.y - z_axis_tool.y() * total_tool_length;
    //     p.z = wp.position.z - z_axis_tool.z() * total_tool_length;

    //     marker.points.push_back(p);
    // }

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
    double width = 0.01;
    double length = 0.02;    
    double center_offset[3] = {0.2, 0.0, 0.25};        //{0.20, 0.0, 0.31};
    double tool_length = 0.046;  // Khoảng bù công cụ 0.035

    // --- 2. Thông số dập lực (Stamping) ---
    double hover_dist = 0.003; // Chờ cách bề mặt 3mm (0.003m)
    double press_dist = 0.003; // Đâm lún sâu vào 5mm (0.005m)
    
    // Tăng step_size lên 1cm (0.01m) để thưa ra. Nếu để 5mm robot dập sẽ rất lâu.
    double step_size = 0.005;   
    bool left_to_right = true;

    // Quét lưới dọc trục Y
    for (double y_local =  - width / 2; y_local <= width / 2 + 1e-5; y_local += step_size) {

        std::vector<double> row_x_points;
        for (double x = - length / 2; x <= length / 2 + 1e-5; x += step_size) {
            row_x_points.push_back(x);
        }

        if (!left_to_right) {
            std::reverse(row_x_points.begin(), row_x_points.end());
        }
        left_to_right = !left_to_right;

        // Xử lý từng điểm trên bề mặt
        for (double x_local : row_x_points) {

            double z_local = 0.0;

            // Tọa độ gốc ngay trên bề mặt phôi (chưa tính bù dao)
            double base_x = center_offset[0] + x_local;
            double base_y = center_offset[1] + y_local;
            double base_z = center_offset[2] + z_local;

            tf2::Vector3 normal_vector(0.0, 0.0, 1.0);

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
            tf2::Quaternion q_urdf_compensation;
            q_urdf_compensation.setRPY(M_PI, 0.0, 0.0);
            
            tf2::Quaternion quat_base;
            rotation_matrix.getRotation(quat_base);

            tf2::Quaternion q_straight_corrected = quat_base * q_urdf_compensation;
            q_straight_corrected.normalize();

            // --- 3. TẠO COMBO 3 ĐIỂM STAMPING ---
            geometry_msgs::Pose hover_pose, press_pose;
            
            hover_pose.orientation.x = q_straight_corrected.x();
            hover_pose.orientation.y = q_straight_corrected.y();
            hover_pose.orientation.z = q_straight_corrected.z();
            hover_pose.orientation.w = q_straight_corrected.w();
            press_pose.orientation = hover_pose.orientation;

            // Define exactly where the physical needle tip should be
            tf2::Vector3 tooltip_straight_hover(base_x, base_y, base_z + hover_dist);
            tf2::Vector3 tooltip_straight_press(base_x, base_y, base_z - press_dist);

            // Flange Target = Tooltip Target - (Tool Z Axis * Tool Length)
            hover_pose.position.x = tooltip_straight_hover.x() ;
            hover_pose.position.y = tooltip_straight_hover.y() ;
            hover_pose.position.z = tooltip_straight_hover.z() ;

            press_pose.position.x = tooltip_straight_press.x() ;
            press_pose.position.y = tooltip_straight_press.y() ;
            press_pose.position.z = tooltip_straight_press.z() ;

            waypoints.push_back(hover_pose);
            waypoints.push_back(press_pose);
            waypoints.push_back(hover_pose);

            double tilt_angle_deg = 15.0; 
            double tilt_angle_rad = tilt_angle_deg * M_PI / 180.0;
            std::vector<double> cone_azimuths_deg = {0.0, 45.0, 135.0, 180.0, 225.0, 315.0};

            for (double azimuth : cone_azimuths_deg) {
                double azimuth_rad = azimuth * M_PI / 180.0;

                tf2::Quaternion q_yaw;
                q_yaw.setRPY(0.0, 0.0, azimuth_rad); 

                tf2::Quaternion q_tilt;
                q_tilt.setRPY(tilt_angle_rad, 0.0, 0.0);

                tf2::Quaternion q_cone_local = q_yaw * q_tilt;
                //tf2::Quaternion q_final = quat_base * q_cone_local;
                // Nhân thêm q_urdf_compensation ở cuối chuỗi ma trận xoay
                tf2::Quaternion q_final = quat_base * q_cone_local * q_urdf_compensation;
                q_final.normalize();

                tf2::Matrix3x3 final_matrix(q_final);
                tf2::Vector3 new_z_direction = final_matrix.getColumn(2); 

                geometry_msgs::Pose tilt_hover, tilt_press;
                tilt_hover.orientation.x = q_final.x();
                tilt_hover.orientation.y = q_final.y();
                tilt_hover.orientation.z = q_final.z();
                tilt_hover.orientation.w = q_final.w();
                tilt_press.orientation = tilt_hover.orientation;

                // 1. Lấy tâm bề mặt phôi làm Điểm Neo (Anchor Point)
                tf2::Vector3 surface_target(base_x, base_y, base_z);

                // Trục new_z_direction đang có chiều đâm từ trên xuống phôi
                
                // 2. Điểm Hover: Lùi chóp kim về phía sau dọc theo trục nghiêng (ngược chiều Z)
                tf2::Vector3 tooltip_tilt_hover(
                    surface_target.x() - new_z_direction.x() * hover_dist,
                    surface_target.y() - new_z_direction.y() * hover_dist,
                    surface_target.z() - new_z_direction.z() * hover_dist
                );

                // 3. Điểm Press: Đâm lún chóp kim về phía trước dọc theo trục nghiêng (cùng chiều Z)
                tf2::Vector3 tooltip_tilt_press(
                    surface_target.x() + new_z_direction.x() * press_dist,
                    surface_target.y() + new_z_direction.y() * press_dist,
                    surface_target.z() + new_z_direction.z() * press_dist
                );

                // Offset the flange using the newly tilted Z-direction vector
                tilt_hover.position.x = tooltip_tilt_hover.x() ;
                tilt_hover.position.y = tooltip_tilt_hover.y() ;
                tilt_hover.position.z = tooltip_tilt_hover.z() ;

                tilt_press.position.x = tooltip_tilt_press.x() ;
                tilt_press.position.y = tooltip_tilt_press.y() ;
                tilt_press.position.z = tooltip_tilt_press.z() ;

                waypoints.push_back(tilt_hover);
                waypoints.push_back(tilt_press);
                waypoints.push_back(tilt_hover);
            }
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
    move_group.setEndEffectorLink("robot_tcp");

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

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // bool success_joint = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // if (success_joint){
    //     move_group.execute(my_plan);
    //     ROS_INFO("Move to initial position");
    // }

    move_group.move();
    ros::Duration(2.0).sleep();


    std::vector<geometry_msgs::Pose> generated_trajectory = generateStampingWaypoints();
    ROS_INFO("Successfully generated %lu waypoints with automatic normal orientation using C++!", 
             generated_trajectory.size());

    move_group.setStartStateToCurrentState();

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
    ros::Duration(2.0).sleep();

    while (ros::ok())
    {
        drawPathMarker(generated_trajectory, marker_pub);
        move_group.setStartStateToCurrentState();

        ROS_INFO("Starting execution of %lu stamping cycles (chunked)...", generated_trajectory.size() / 3);

        for (size_t i = 0; i < generated_trajectory.size(); i += 3) {
            
            if (i + 2 >= generated_trajectory.size()) break;

            // =====================================================================
            // BƯỚC 1: DÙNG MÀNG LỌC KHÔNG GIAN TRƯỚC KHI GỌI PTP
            // =====================================================================
            geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
            
            double dx = current_pose.position.x - generated_trajectory[i].position.x;
            double dy = current_pose.position.y - generated_trajectory[i].position.y;
            double dz = current_pose.position.z - generated_trajectory[i].position.z;
            double pos_distance = std::sqrt(dx*dx + dy*dy + dz*dz);

            tf2::Quaternion q_curr, q_target;
            tf2::fromMsg(current_pose.orientation, q_curr);
            tf2::fromMsg(generated_trajectory[i].orientation, q_target);
            double angle_diff = q_curr.angleShortestPath(q_target);

            // Nếu cách xa hơn 1mm HOẶC lệch góc quá 0.01 Radian thì mới chạy PTP
            if (pos_distance > 0.001 || angle_diff > 0.01) {
                move_group.setPoseTarget(generated_trajectory[i]);
                moveit::planning_interface::MoveGroupInterface::Plan ptp_plan;
                
                if (move_group.plan(ptp_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                    move_group.execute(ptp_plan); 
                    ros::Duration(1.0).sleep(); 
                } else {
                    ROS_WARN("Cycle %zu failed: Could not PTP to Hover pose.", i/3);
                    continue; 
                }
            } else {
                ROS_INFO("Cycle %zu: Already at target position. Skipping PTP to avoid hardware freeze.", i/3);
            }

            move_group.setStartStateToCurrentState(); 

            // =====================================================================
            // BƯỚC 2: CHUYỂN ĐỘNG CARTESIAN DẬP ĐỀU TỐC (BYPASS IPTP)
            // =====================================================================
            std::vector<geometry_msgs::Pose> stamp_cycle = {
                generated_trajectory[i+1],   // Press
                generated_trajectory[i+2]    // Hover
            };

            moveit_msgs::RobotTrajectory trajectory;
            moveit_msgs::MoveItErrorCodes error_code;
            const double eef_step = 0.005; 

            double fraction = move_group.computeCartesianPath(stamp_cycle, eef_step, trajectory, true, &error_code);

            if (fraction > 0.9) {
                ROS_INFO("Cycle %zu: Cartesian path planned. Applying deterministic time...", i/3);
                
                // Thuật toán gán thời gian tĩnh (Triệt tiêu 100% bug treo IPTP)
                double speed_m_s = 0.02; // Tốc độ dập 20 mm/s
                double dt = eef_step / speed_m_s; // Thời gian cho mỗi mốc 1mm (0.05s)
                double current_time = 0.0;

                // Cấu hình điểm gốc
                trajectory.joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
                trajectory.joint_trajectory.points[0].velocities.assign(6, 0.0);
                trajectory.joint_trajectory.points[0].accelerations.assign(6, 0.0);

                // Cấp phát thời gian và vận tốc rỗng (Yaskawa yêu cầu cấu trúc mảng đầy đủ)
                for (size_t j = 1; j < trajectory.joint_trajectory.points.size(); ++j) {
                    current_time += dt;
                    trajectory.joint_trajectory.points[j].time_from_start = ros::Duration(current_time);
                    trajectory.joint_trajectory.points[j].velocities.assign(6, 0.0);
                    trajectory.joint_trajectory.points[j].accelerations.assign(6, 0.0);
                }

                moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
                cartesian_plan.trajectory_ = trajectory;
                
                ROS_INFO("Cycle %zu: Sending Cartesian trajectory to Yaskawa driver...", i/3);
                moveit::core::MoveItErrorCode result = move_group.execute(cartesian_plan);
                
                if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                    ROS_ERROR("Execution failed at cycle %zu with error code: %d", i/3, result.val);
                    break;
                }
                
                ROS_INFO("Cycle %zu: Cartesian execution completed.", i/3);
                ros::Duration(1.0).sleep(); 
                move_group.setStartStateToCurrentState(); 

            } else {
                ROS_WARN("Cycle failed at index %zu during Cartesian stamping. Fraction: %f", i/3, fraction);
            }
        }

        ROS_INFO("Completed the entire stamping grid. Resting for 2 seconds before looping...");
        ros::Duration(2.0).sleep();
    }
    
    ROS_INFO("Stopping robot controller gracefully...");
    spinner.stop();
    ros::shutdown();
    return 0;
    
    
}