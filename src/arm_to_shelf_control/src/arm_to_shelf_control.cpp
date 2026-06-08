#include <rclcpp/rclcpp.hpp>
// 順手將 .h 改為 ROS 2 現代化的 .hpp 標頭檔
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/command.hpp>
#include <geometry_msgs/msg/pose.hpp> // 補上遺漏的標頭檔
#include <std_srvs/srv/trigger.hpp>
#include <vision_interfaces/srv/armcoodinate.hpp>
#include <vision_interfaces/srv/shelf_coodinate.hpp>
#include <vision_interfaces/srv/twist_moveit_servo.hpp>
#include <std_msgs/msg/bool.hpp>

#include <thread>
#include <memory>
#include <string>
#include <chrono>

using namespace std::chrono_literals;
using namespace moveit_servo;

static const std::string ARM_GROUP = "arm";
static const std::string GRIPPER_GROUP = "gripper";

class arm_to_shelf_control : public rclcpp::Node
{
private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_run_service_;

    rclcpp::Client<vision_interfaces::srv::Armcoodinate>::SharedPtr view_shelf_coord_client_;
    rclcpp::Client<vision_interfaces::srv::ShelfCoodinate>::SharedPtr shelf_coord_client_;
    rclcpp::Client<vision_interfaces::srv::TwistMoveitServo>::SharedPtr servo_twist_client_;

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr moveit_servo_publisher_;

    rclcpp::Duration elapsed_time;
    rclcpp::Time shelf_feature_time;
    double shelf_vel;

public:
    arm_to_shelf_control();
    ~arm_to_shelf_control();
    void run();
    void run_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void moveit_init();
    void move_to_view_shelf_pose();
    void move_to_shelf_pose();
    bool arm_planner(geometry_msgs::msg::Pose &target_pose, std::string state = "normal", double cli_used_time = 0.0);
    bool gripper_planner(std::string target = "close");
    bool moveit_servo_move(geometry_msgs::msg::TwistStamped &twist_pub, geometry_msgs::msg::PoseStamped &init_pose, double offset = 0.0, std::string status = "put_in"); // offset 是為了往下幾毫米
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 1. 建立節點實例
    auto node = std::make_shared<arm_to_shelf_control>();
    node->moveit_init();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

arm_to_shelf_control::arm_to_shelf_control()
    : Node("move_to_shelf"), elapsed_time(0, 0)
{
    this->view_shelf_coord_client_ = this->create_client<vision_interfaces::srv::Armcoodinate>("view_shelf_coord");
    this->shelf_coord_client_ = this->create_client<vision_interfaces::srv::ShelfCoodinate>("shelf_coord");
    this->servo_twist_client_ = this->create_client<vision_interfaces::srv::TwistMoveitServo>("servo_twist");
    this->trigger_run_service_ = this->create_service<std_srvs::srv::Trigger>("run_service", std::bind(&arm_to_shelf_control::run_callback, this, std::placeholders::_1, std::placeholders::_2));
    this->moveit_servo_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("delta_twist_cmds", 10);

    // elapsed_time = rclcpp::Duration(0, 0);
}

arm_to_shelf_control::~arm_to_shelf_control()
{
}

void arm_to_shelf_control::run()
{
    const double place_distance = 0.004; // 單位mm 放置距離4mm
    const double place_vel = 0.01;       // 速度 每秒1cm

    move_to_view_shelf_pose();

    // ---------------- 第一步執行到看架子的點------------------

    std::this_thread::sleep_for(3s); // 等待一秒確保 python端可以偵測到shelf pose 已經完成動作

    move_to_shelf_pose();
    auto reset_pub = this->create_publisher<std_msgs::msg::Bool>("update_robot_state", rclcpp::SystemDefaultsQoS());
    std_msgs::msg::Bool reset_msg;
    reset_msg.data = true;
    reset_pub->publish(reset_msg);

    // ---------------- 第二步執行到架子未來位置------------------
    rclcpp::Time now = this->now();
    bool suc = false;
    while (now < this->shelf_feature_time)
    {
        now = this->now();
        if (!suc)
            this->move_group_->setStartStateToCurrentState();
    }
    RCLCPP_INFO(this->get_logger(), "時間到架子與手臂可以平行執行");

    // 接下來去twist
    std::string status = "put_in";
    geometry_msgs::msg::TwistStamped twist_req;
    twist_req.header.frame_id = "base_link";
    twist_req.twist.linear.x = abs(place_vel * sin(10 * M_PI / 180));      // 沿 X 軸
    twist_req.twist.linear.y = shelf_vel;                                  // 沿 Y 軸
    twist_req.twist.linear.z = -1 * abs(place_vel * cos(10 * M_PI / 180)); // 沿 Z 軸
    twist_req.twist.angular.x = 0.0;                                       // 沿 X 軸
    twist_req.twist.angular.y = 0.0;                                       // 沿 Y 軸
    twist_req.twist.angular.z = 0.0;                                       // 沿 Z 軸

    geometry_msgs::msg::PoseStamped init_pose = this->move_group_->getCurrentPose("gripper_tcp");
    bool success = moveit_servo_move(twist_req, init_pose, place_distance);
    // ------------------------ 第三步執行 MoveIt Servo 控制手臂跟隨貨架移動 並將輪自放到柱子裡面--------------------

    gripper_planner("open");
    // ------------------------ 第四步夾爪打開放輪子--------------------

    status = "quit";
    twist_req = geometry_msgs::msg::TwistStamped();
    twist_req.header.frame_id = "base_link";
    twist_req.twist.linear.x = -1 * abs(0.05 * sin(10.0 * M_PI / 180)); // 沿 X 軸
    twist_req.twist.linear.y = shelf_vel;                            // 沿 Y 軸
    twist_req.twist.linear.z = abs(0.05 * cos(10.0 * M_PI / 180));      // 沿 Z 軸
    twist_req.twist.angular.x = 0.0;                                 // 沿 X 軸
    twist_req.twist.angular.y = 0.0;                                 // 沿 Y 軸
    twist_req.twist.angular.z = 0.0;                                 // 沿 Z 軸

    init_pose = this->move_group_->getCurrentPose("gripper_tcp");
    success = moveit_servo_move(twist_req, init_pose, 0.0, status);
    // ------------------------ 第五步執行 MoveIt Servo 控制手臂跟隨貨架移動 並將輪子從柱子裡面拉出來--------------------

    std::this_thread::sleep_for(2s);
    geometry_msgs::msg::Pose origin_pose;
    origin_pose.position.x = 0.287;
    origin_pose.position.y = 0.000;
    origin_pose.position.z = 0.287;
    origin_pose.orientation.x = 0.707;
    origin_pose.orientation.y = 0.707;
    origin_pose.orientation.z = 0.000;
    origin_pose.orientation.w = 0.000;
    arm_planner(origin_pose); // 最後回到初始位置
    gripper_planner("close"); // 最後回到初始位置
}

void arm_to_shelf_control::run_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request; // 目前沒有使用 request 的內容，可以先這樣避免編譯警告
    RCLCPP_INFO(this->get_logger(), "Received run command, starting the process...");
    std::thread([this]()
                { this->run(); })
        .detach();

    response->success = true;
    response->message = "Process completed successfully.";
}

void arm_to_shelf_control::moveit_init()
{
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    gripper_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
}

void arm_to_shelf_control::move_to_view_shelf_pose()
{
    auto view_shelf_coord_request = std::make_shared<vision_interfaces::srv::Armcoodinate::Request>();
    view_shelf_coord_request->result = "get_view_shelf_coord";
    while (!view_shelf_coord_client_->wait_for_service()) // 無限等待服務可用
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto view_shelf_coord_reponse = view_shelf_coord_client_->async_send_request(view_shelf_coord_request);

    auto response = view_shelf_coord_reponse.get();
    arm_planner(response->arm_cood);
}

void arm_to_shelf_control::move_to_shelf_pose()
{
    auto shelf_coord_request = std::make_shared<vision_interfaces::srv::ShelfCoodinate::Request>();
    shelf_coord_request->req_cmd = "get_shelf_cood";

    while (!shelf_coord_client_->wait_for_service())
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto shelf_coord_reponse = shelf_coord_client_->async_send_request(shelf_coord_request);

    auto response = shelf_coord_reponse.get();

    rclcpp::Time start_cli = response->start_pos_time;
    this->shelf_feature_time = start_cli;
    std::string status = response->status_message;
    shelf_vel = response->shelf_vel;

    rclcpp::Time end_cli = this->now();

    elapsed_time = end_cli - start_cli;

    bool success = arm_planner(response->shelf_pose, "feature_postion", elapsed_time.seconds()); // 寫到這邊就已經讓夾爪移到架子未來位置
    // bool success = arm_planner(response->shelf_pose); // 寫到這邊就已經讓夾爪移到架子未來位置
}

bool arm_to_shelf_control::arm_planner(geometry_msgs::msg::Pose &target_pose, std::string state, double cli_used_time)
{
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    if (state == "normal")
    {
        move_group_->setPoseTarget(target_pose);
        bool success = (move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "手臂規劃失敗");
            return false;
        }
    }
    else if (state == "feature_postion")
    {
        this->move_group_->setPoseTarget(target_pose);  
        bool succes = (this->move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!succes)
        {
            RCLCPP_INFO(this->get_logger(), "規劃錯誤");
        }

        double plan_time = arm_plan.planning_time;

        size_t point_count = arm_plan.trajectory.joint_trajectory.points.size();
        double move_time;
        if (point_count > 0)
        {
            // 2. 取得最後一個軌跡點
            auto last_point = arm_plan.trajectory.joint_trajectory.points.back();

            // 3. 轉為秒數 (包含秒與奈秒)
            move_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;

            RCLCPP_INFO(this->get_logger(), "手臂預估運動時間：%f 秒", move_time);
        }
        else
        {
            move_time = 0;
        }

        RCLCPP_INFO(this->get_logger(), "規劃時間：%f", move_time);

        const double position_offset = 1.0; // 要往後一秒鐘避免撞機
        double t = plan_time * 2 + move_time + abs(cli_used_time) + position_offset;
        RCLCPP_INFO(this->get_logger(), "總時間：%f", t);
        double shelf_move_distance = this->shelf_vel * t; // 單位公尺

        this->shelf_feature_time += rclcpp::Duration::from_seconds(t);
        target_pose.position.y += shelf_move_distance;

        // RCLCPP_INFO(this->get_logger(), "手臂目標座標 X：%f", target_pose.position.x);
        // RCLCPP_INFO(this->get_logger(), "手臂目標座標 Y：%f", target_pose.position.y);
        // RCLCPP_INFO(this->get_logger(), "手臂目標座標 Z：%f", target_pose.position.z);

        this->move_group_->setPoseTarget(target_pose);
        succes = (this->move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!succes)
        {
            RCLCPP_INFO(this->get_logger(), "2次規劃錯誤");
        }
    }
    RCLCPP_INFO(this->get_logger(), "手臂規劃成功，開始執行...");
    // 注意：execute 是阻塞型函式，會一直等到手臂走到定位
    move_group_->execute(arm_plan);
    return true;
}

bool arm_to_shelf_control::gripper_planner(std::string target)
{
    gripper_group_->setNamedTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool success = (gripper_group_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "手臂規劃成功，開始執行...");
        // 注意：execute 是阻塞型函式，會一直等到手臂走到定位
        gripper_group_->execute(gripper_plan);
        return true;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "手臂規劃失敗");
        return false;
    }
}

bool arm_to_shelf_control::moveit_servo_move(geometry_msgs::msg::TwistStamped &twist_pub, geometry_msgs::msg::PoseStamped &init_pose, double offset, std::string status)
{
    this->moveit_servo_publisher_->publish(twist_pub);
    geometry_msgs::msg::PoseStamped current_pose = this->move_group_->getCurrentPose("gripper_tcp");
    if (status == "put_in")
    {
        double offset_x = offset * sin(10.0 * M_PI / 180);
        double offset_z = -offset * cos(10.0 * M_PI / 180);

        while (current_pose.pose.position.x < init_pose.pose.position.x + offset_x || current_pose.pose.position.z > init_pose.pose.position.z + offset_z)
        {
            if (current_pose.pose.position.x >= init_pose.pose.position.x + offset_x)
            {
                twist_pub.twist.linear.x = 0.0;
            }
            if (current_pose.pose.position.z <= init_pose.pose.position.z + offset_z)
            {
                twist_pub.twist.linear.z = 0.0;
            }
            this->moveit_servo_publisher_->publish(twist_pub);
            current_pose = this->move_group_->getCurrentPose("gripper_tcp");
        }

        return true;
    }
    else if (status == "quit")
    {
        while ((fabs(init_pose.pose.position.x - current_pose.pose.position.x) <= 0.05 * sin(10.0 * M_PI / 180.0)) ||
               (fabs(init_pose.pose.position.z - current_pose.pose.position.z) <= 0.05 * cos(10.0 * M_PI / 180.0)))
        {
            this->moveit_servo_publisher_->publish(twist_pub);
            current_pose = this->move_group_->getCurrentPose("gripper_tcp");
            // RCLCPP_INFO(this->get_logger(), "發布twist x: %f ", twist_pub.twist.linear.x);
            // RCLCPP_INFO(this->get_logger(), "發布twist y: %f ", twist_pub.twist.linear.y);
            // RCLCPP_INFO(this->get_logger(), "發布twist z: %f ", twist_pub.twist.linear.z);
        }
        return true;
    }
    else{
        return false;
    }
}
