# ===== Simulation trên RViz =====

Terminal 1: (Bật môi trường ảo bao gồm MoveIt, RViz và Bộ điều khiển giả lập):
    roslaunch motoman_motomini_moveit_config demo.launch

Terminal 2 (Chạy thuật toán):
    rosrun my_robot_controller linear_spline_interpolation.py


# ===== Connect to Motomini =====

Terminal 1 (Kết nối Driver phần cứng):
    roslaunch motoman_motomini_support robot_interface_streaming_motomini.launch robot_ip:=192.168.1.12 controller:=yrc1000

Terminal 2 (Nạp bản vẽ và Bật bộ não MoveIt):
    roslaunch motoman_motomini_moveit_config planning_context.launch load_robot_description:=true

    roslaunch motoman_motomini_moveit_config move_group.launch

Terminal 3 (Mở mắt thần RViz giám sát):
    roslaunch motoman_motomini_moveit_config moveit_rviz.launch config:=true

Terminal 4 (Cấp điện Servo và Chạy Code):
    rosservice call /robot_enable

    rosrun my_robot_controller linear_spline_interpolation.py

- Khắc phục lỗi Global status: No TF data
    rosrun robot_state_publisher robot_state_publisher


# === Sửa lỗi đồng bộ Time ===

timedatectl set-local-rtc 1 --adjust-system-clock

timedatectl

cd ~/catkin_ws
rm -rf build/ devel/
catkin_make


# === CÁC TẬP LỆNH THAM KHẢO class MoveGroupInterface ===

1. # Nhóm Cài đặt Hệ thống (System Configuration)
Nhóm lệnh này thường được gọi ngay sau khi khởi tạo đối tượng move_group, nhằm thiết lập các giới hạn an toàn và hệ quy chiếu trước khi robot MotoMINI thực sự chuyển động.

- setMaxVelocityScalingFactor(double factor) và setMaxAccelerationScalingFactor(double factor): Cắt giảm tốc độ và gia tốc tối đa. Tham số truyền vào từ 0.01 đến 1.0 (tương ứng 1% đến 100%). Luôn nên để 0.1 khi test trên phần cứng thật.

- setPoseReferenceFrame(const std::string &frame_name): Chỉ định gốc tọa độ tham chiếu. Thường được đặt là "world" hoặc "base_link". Toàn bộ tọa độ X, Y, Z sau này sẽ được tính từ gốc này.

- setGoalPositionTolerance(double tolerance) và setGoalOrientationTolerance(double tolerance): Cho phép hệ thống có một sai số nhỏ (tính bằng mét hoặc radian) khi tiếp cận mục tiêu, giúp tránh lỗi "Time out" khi bộ giải IK không tìm được tọa độ tuyệt đối 100%.


// Limit speed to 10% for physical safety
    move_group.setMaxVelocityScalingFactor(0.1);
    
    // Set coordinate frame to the robot's base
    move_group.setPoseReferenceFrame("base_link");
    
    // Allow 1cm error in position and 0.05 rad error in rotation
    move_group.setGoalPositionTolerance(0.01);
    move_group.setGoalOrientationTolerance(0.05);


2. # Nhóm Thiết lập Mục tiêu (Target Setting)
Nhóm lệnh này có chức năng nạp tọa độ đích vào bộ nhớ tạm. Chúng không làm robot di chuyển.

- setJointValueTarget(const std::vector<double> &joint_values): Sử dụng Động học thuận. Bạn nạp mảng chứa 6 giá trị góc (radian). Rất phù hợp để đưa cánh tay về tư thế Home hoặc các vị trí chờ cố định.

- setPoseTarget(const geometry_msgs::Pose &target): Sử dụng Động học ngược. Bạn nạp tọa độ không gian (X, Y, Z và Quaternion). Đây là lệnh chủ lực để đưa mũi gắp đến các vị trí linh hoạt, ví dụ như tọa độ ngẫu nhiên của các chai men vi sinh được trả về từ camera.


// Target 1: Joint space (Forward Kinematics)
    std::vector<double> home_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    `move_group.setJointValueTarget(home_joints);`

    // Target 2: Cartesian space (Inverse Kinematics)
    geometry_msgs::Pose pick_pose;
    pick_pose.position.x = 0.35;
    pick_pose.position.y = 0.15;
    pick_pose.position.z = 0.20;
    // Assume orientation is already set correctly
    move_group.setPoseTarget(pick_pose);


3. # Nhóm Quy hoạch và Thực thi (Planning & Execution)
Nhóm lệnh này là bộ não của MoveIt, chuyên gọi các thuật toán giải toán và giao tiếp với phần cứng.

- plan(MoveGroupInterface::Plan &plan): Yêu cầu hệ thống giải Động học ngược và dò đường đi. Hàm trả về kiểu boolean (True nếu tìm được đường, False nếu thất bại). Kết quả quỹ đạo được nạp vào biến tham chiếu.

- execute(const MoveGroupInterface::Plan &plan): Gửi trực tiếp bản quy hoạch quỹ đạo xuống tủ điều khiển để robot chạy.

- move(): Lệnh chạy tự động. Nó tự động gọi hàm plan, nếu thành công sẽ gọi tiếp execute.


    `moveit::planning_interface::MoveGroupInterface::Plan my_plan;`
    
    // Generate the trajectory
    `bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);`
    
    // Send to hardware if planning is collision-free
    `if (success) {
        move_group.execute(my_plan);`
    }


4. # Nhóm Quy hoạch Tuyến tính (Cartesian Path)
Khi gắp và xếp vật vào khay, bạn không thể để robot đi theo đường vòng cung tự do, mà phải ép nó đi theo một đường thẳng tuyệt đối từ trên xuống dưới.

- computeCartesianPath(const std::vector<geometry_msgs::Pose> &waypoints, double eef_step, double jump_threshold, trajectory_msgs::JointTrajectory &trajectory): Nội suy một dải tọa độ để ép mũi gắp bám sát biên dạng. Hàm này trả về một con số thập phân từ 0.0 đến 1.0 (tương ứng 0% đến 100% tỷ lệ đoạn đường hoàn thành mà không bị va chạm).


    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(current_pose); // Start point
    
    // Create a strict vertical downward movement of 10 cm
    geometry_msgs::Pose target_pose = current_pose;
    target_pose.position.z -= 0.10; 
    waypoints.push_back(target_pose); // End point

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01; // Interpolate a point every 1 cm
    
    // Compute the straight path
    `double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);`


5. # **Nhóm Đọc Trạng thái (State Retrieval)**
Nhóm lệnh này dùng để trích xuất dữ liệu thực tế từ các cảm biến Encoder của động cơ, phục vụ cho việc tính toán bù trừ hoặc gỡ lỗi.

- getCurrentPose(): Trả về struct geometry_msgs::PoseStamped chứa thông tin tọa độ X, Y, Z và góc xoay của mũi gắp tại thời điểm gọi hàm.

- getCurrentJointValues(): Trả về mảng std::vector<double> chứa 6 góc xoay thực tế của 6 khớp động cơ.

- setStartStateToCurrentState(): Lấy trạng thái vật lý thực tại của robot để làm điểm khởi đầu cho chu kỳ tính toán tiếp theo. Cực kỳ quan trọng khi chạy vòng lặp liên tục để tránh hiện tượng báo lỗi kẹt quỹ đạo.


    // Read current 3D position
    `geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;`

    // Read current motor angles
    `std::vector<double> current_angles = move_group.getCurrentJointValues();`

    // Reset starting logic to physical state
    `move_group.setStartStateToCurrentState();`


# === Cách trích xuất dữ liệu vận tốc, gia tốc từ biến my_plan sau khi gọi hàm .plan() ===

1. Cấu trúc "Búp bê Nga" của biến my_planTrong C++, kiểu dữ liệu Plan được cấu tạo lồng ghép thành nhiều tầng (giống như búp bê Nga). Để lấy được con số gia tốc, bạn phải đi qua đúng đường dẫn tĩnh này:
    my_plan -> trajectory -> joint_trajectory -> points -> velocities / accelerations

2. Đoạn Code C++ trích xuất dữ liệu thực tế
Bạn có thể chèn trực tiếp đoạn code này vào ngay bên dưới dòng lệnh bool success_joint = (move_group.plan(my_plan) == ...SUCCESS); trong file point_to_point.cpp của bạn:

`if (success_joint) {
        // 1. Tạo một "đường tắt" (Reference) trỏ thẳng vào mảng các điểm quỹ đạo để tránh gõ dài
        const std::vector<trajectory_msgs::JointTrajectoryPoint>& traj_points = my_plan.trajectory_.joint_trajectory.points;

        ROS_INFO("Quy hoach thanh cong! Tong so diem noi suy: %zu", traj_points.size());

        // 2. Dùng vòng lặp duyệt qua các điểm. (Ở đây ta chỉ in điểm đầu, giữa và cuối để tránh trôi Terminal)
        for (size_t i = 0; i < traj_points.size(); ++i) {
            
            if (i == 0 || i == traj_points.size() / 2 || i == traj_points.size() - 1) {
                
                // Lấy reference của từng điểm cụ thể
                const auto& point = traj_points[i];
                
                // Trích xuất thời gian dự kiến (tính từ lúc bắt đầu chạy)
                double time_sec = point.time_from_start.toSec();

                // Lấy vận tốc và gia tốc của khớp số 1 (Trục S/Base) - Index trong mảng C++ luôn bắt đầu từ 0
                double vel_j1 = point.velocities[0];
                double acc_j1 = point.accelerations[0];

                ROS_INFO("Diem %zu (Giay thu %.3f) | Khop 1 - Van toc: %.3f rad/s, Gia toc: %.3f rad/s^2",
                         i, time_sec, vel_j1, acc_j1);
            }
        }
        
        // Sau khi kiem tra thong so an toan xong moi cho thuc thi
        // move_group.execute(my_plan); 
    }`


- traj_points.size(): Số lượng điểm mà MoveIt băm nhỏ ra (Ví dụ: 85 điểm). Nếu bạn di chuyển đoạn đường dài nhưng số điểm nội suy quá ít (dưới 10 điểm), robot đi sẽ bị giật cục.

- time_from_start: Giúp bạn biết chính xác hệ thống mất bao nhiêu giây để hoàn thành lệnh gắp này (Ví dụ: 2.1 giây). Dữ liệu này dùng để tính toán năng suất chu kỳ (Cycle Time) xem có đạt KPI của dây chuyền phân loại hay không.

- velocities và accelerations: Tại điểm khởi đầu (Điểm 0) và điểm kết thúc (Điểm 84), vận tốc lý tưởng nhất luôn phải bằng 0.0. Ở khúc giữa quỹ đạo (Điểm 42), vận tốc sẽ đạt đỉnh (peak) và gia tốc sẽ bắt đầu giảm về âm (để phanh lại).


# === ROSBAG ===

1. Ghi dữ liệu (rosbag record)

    `rosbag record /tf /joint_states -O my_robot_run.bag`

- rosbag record: Lệnh yêu cầu bắt đầu ghi.

- /tf /joint_states: Danh sách các topic cụ thể bạn muốn lưu. Nếu muốn ghi toàn bộ các topic đang có trong hệ thống, bạn có thể thay thế bằng tham số -a (All).

- -O my_robot_run.bag: Lưu thành file tên là my_robot_run.bag. Nếu không có tham số này, ROS sẽ tự đặt tên file theo mốc ngày giờ năm tháng.

Để dừng quá trình ghi, bạn chỉ cần nhấn Ctrl + C tại Terminal đó.

2. Xem thông tin file dữ liệu (rosbag info)

    `rosbag info my_robot_run.bag`

3. Phát lại dữ liệu (rosbag play)
- Kịch bản: Bạn tắt toàn bộ robot thật, tắt các node điều khiển quỹ đạo, chỉ bật duy nhất phần mềm mô phỏng RViz lên. Bạn muốn xem lại chuyển động của robot dựa trên file dữ liệu đã ghi.

    `rosbag play my_robot_run.bag`

Lúc này, phần mềm RViz sẽ nhận được các gói tin /tf đổ về từ file bag và dựng lại chính xác chuyển động của cánh tay robot trên màn hình y hệt như lúc nó đang vận hành trong thực tế.

- Phát nhanh/chậm tốc độ: Nếu file dữ liệu quá dài và bạn muốn tua nhanh, bạn dùng tham số -r (rate). Ví dụ, phát nhanh gấp đôi:
    `rosbag play my_robot_run.bag -r 2.0`

- Tạm dừng dữ liệu: Trong lúc lệnh rosbag play đang chạy, bạn có thể nhấn phím Spacebar (Phím cách) trên bàn phím để tạm dừng dữ liệu tại một thời điểm nhằm soi kỹ tọa độ, và nhấn lại Spacebar để tiếp tục chạy.