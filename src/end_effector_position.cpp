#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "end_effector_data_logger");
    ros::NodeHandle nh;

    // 1. Create a tf2 buffer and a listener to subscribe to the /tf topic
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // 2. Set the data logging frequency (10 Hz)
    ros::Rate rate(1.0);

    ROS_INFO("Starting real-time data collection for End-Effector...");

    // Allow TF buffer to warm up and accumulate coordinate data from network
    ros::Duration(2.0).sleep();

    while (nh.ok()) {
        geometry_msgs::TransformStamped transformStamped;

        try {
            // HARDWARE FIX: Reverted target_frame back to "base_link" as per hardware constraint
            // Increased timeout to 0.3s to tolerate industrial controller communication latency
            transformStamped = tfBuffer.lookupTransform("base_link", "robot_tcp", ros::Time(0), ros::Duration(0.3));
            
            // 3. Extract Translation (X, Y, Z)
            double x = transformStamped.transform.translation.x;
            double y = transformStamped.transform.translation.y;
            double z = transformStamped.transform.translation.z;

            // 4. Extract Rotation (Quaternion) and convert to Euler angles (Roll, Pitch, Yaw)
            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w
            );
            
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw); // Conversion to Radians

            // 5. Print the real-time data to log console
            ROS_INFO("Data: X:%.3f Y:%.3f Z:%.3f | R:%.3f P:%.3f Y:%.3f", 
                     x, y, z, roll, pitch, yaw);

        } catch (tf2::TransformException &ex) {
            // Using throttle to prevent log flooding while waiting for the robot state publisher to turn on
            ROS_WARN_THROTTLE(2.0, "Waiting for TF tree connection... Error: %s", ex.what());
            rate.sleep();
            continue;
        }

        rate.sleep();
    }

    return 0;
}