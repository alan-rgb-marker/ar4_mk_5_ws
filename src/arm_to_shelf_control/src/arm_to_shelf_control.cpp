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

    // rclcpp::Time shelf_feature_time;
    float shelf_vel;

public:
    arm_to_shelf_control();

    ~arm_to_shelf_control();

    void run();

    void run_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void moveit_init();

    void move_to_view_shelf_pose();

    void move_to_shelf_pose();

    bool arm_planner(geometry_msgs::msg::Pose &target_pose, std::string state = "normal");

    bool gripper_planner(std::string target = "close");

    bool moveit_servo_twist_client(const geometry_msgs::msg::TwistStamped &twist_req, std::string status = "docking");

    bool moveit_servo_move(geometry_msgs::msg::TwistStamped &twist_cmd);
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
    : Node("move_to_shelf")
{
    view_shelf_coord_client_ = this->create_client<vision_interfaces::srv::Armcoodinate>("view_shelf_coord");
    shelf_coord_client_ = this->create_client<vision_interfaces::srv::ShelfCoodinate>("shelf_coord");
    servo_twist_client_ = this->create_client<vision_interfaces::srv::TwistMoveitServo>("servo_twist");
    trigger_run_service_ = this->create_service<std_srvs::srv::Trigger>("run_service", std::bind(&arm_to_shelf_control::run_callback, this, std::placeholders::_1, std::placeholders::_2));
}

arm_to_shelf_control::~arm_to_shelf_control()
{
}

void arm_to_shelf_control::run()
{
    move_to_view_shelf_pose();

    // ---------------- 第一步執行到看架子的點------------------

    std::this_thread::sleep_for(7s); // 等待一秒確保 python端可以偵測到shelf pose 已經完成動作

    move_to_shelf_pose();

    // ---------------- 第二步執行到架子未來位置------------------
    // rclcpp::Time now = this->now();
    // while (now < shelf_feature_time)
    // {
    //     now = this->now();
    // }

    // 接下來去twist
    std::string status = "docking";
    geometry_msgs::msg::TwistStamped twist_req;
    twist_req.header.frame_id = "base_link";
    twist_req.twist.linear.x = abs(0.03 * sin(10 * M_PI / 180));      // 沿 X 軸
    twist_req.twist.linear.y = shelf_vel;                             // 沿 Y 軸
    twist_req.twist.linear.z = -1 * abs(0.03 * cos(10 * M_PI / 180)); // 沿 Z 軸
    twist_req.twist.angular.x = 0.0;                                  // 沿 X 軸
    twist_req.twist.angular.y = 0.0;                                  // 沿 Y 軸
    twist_req.twist.angular.z = 0.0;                                  // 沿 Z 軸
    bool success = moveit_servo_twist_client(twist_req, status);
    // ------------------------ 第三步執行 MoveIt Servo 控制手臂跟隨貨架移動 並將輪自放到柱子裡面--------------------

    gripper_planner("open");
    // ------------------------ 第四步夾爪打開放輪子--------------------

    status = "quiting";
    twist_req = geometry_msgs::msg::TwistStamped();
    twist_req.header.frame_id = "base_link";
    twist_req.twist.linear.x = -1 * abs(0.3 * sin(10 * M_PI / 180)); // 沿 X 軸
    twist_req.twist.linear.y = shelf_vel;                            // 沿 Y 軸
    twist_req.twist.linear.z = abs(0.3 * cos(10 * M_PI / 180));      // 沿 Z 軸
    twist_req.twist.angular.x = 0.0;                                 // 沿 X 軸
    twist_req.twist.angular.y = 0.0;                                 // 沿 Y 軸
    twist_req.twist.angular.z = 0.0;                                 // 沿 Z 軸
    success = moveit_servo_twist_client(twist_req, status);
    // ------------------------ 第五步執行 MoveIt Servo 控制手臂跟隨貨架移動 並將輪子從柱子裡面拉出來--------------------
    geometry_msgs::msg::Pose init_pose = geometry_msgs::msg::Pose();
    arm_planner(init_pose); // 最後回到初始位置
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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "執行到這裡1");
    while (!shelf_coord_client_->wait_for_service())
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "執行到這裡2");
    
    auto shelf_coord_reponse = shelf_coord_client_->async_send_request(shelf_coord_request);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "執行到這裡3");

    auto response = shelf_coord_reponse.get();

    // shelf_feature_time = response->future_time;
    std::string status = response->status_message;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "執行到這裡");
    shelf_vel = response->shelf_vel;
    
    // bool success = arm_planner(response->shelf_pose, "time"); // 寫到這邊就已經讓夾爪移到架子未來位置
    bool success = arm_planner(response->shelf_pose); // 寫到這邊就已經讓夾爪移到架子未來位置
}

bool arm_to_shelf_control::arm_planner(geometry_msgs::msg::Pose &target_pose, std::string state)
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
    else if (state == "time")
    {
        float distance;
        do
        {
            target_pose.position.y -= 0.05;
            move_group_->setPoseTarget(target_pose);
            bool success = (move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            double t = arm_plan.trajectory.joint_trajectory.points.back().time_from_start.sec;
            distance = shelf_vel * t;
        } while (distance > 0.05);
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

bool arm_to_shelf_control::moveit_servo_twist_client(const geometry_msgs::msg::TwistStamped &twist_req, std::string status)
{
    auto twist_moveit_servo_request = std::make_shared<vision_interfaces::srv::TwistMoveitServo::Request>();

    twist_moveit_servo_request->twist_req = twist_req;
    twist_moveit_servo_request->status = status;
    while (!servo_twist_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto twist_moveit_servo_reponse = servo_twist_client_->async_send_request(twist_moveit_servo_request);

    auto response = twist_moveit_servo_reponse.get();
    if (response->status != "get")
    {
        RCLCPP_INFO(this->get_logger(), "MoveIt Servo 回應不是 get: %s", response->status.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "MoveIt Servo 穩定，取得回應。");
    }

    return true;
}

bool arm_to_shelf_control::moveit_servo_move(geometry_msgs::msg::TwistStamped &twist_req)
{
    auto twist_moveit_servo_request = std::make_shared<vision_interfaces::srv::TwistMoveitServo::Request>();

    twist_moveit_servo_request->twist_req = twist_req;
    while (!servo_twist_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto twist_moveit_servo_reponse = servo_twist_client_->async_send_request(twist_moveit_servo_request);

    auto response = twist_moveit_servo_reponse.get();
    if (response->status != "get")
    {
        RCLCPP_INFO(this->get_logger(), "MoveIt Servo 回應不是 get: %s", response->status.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "MoveIt Servo 穩定，取得回應。");
    }
    return true;
}
