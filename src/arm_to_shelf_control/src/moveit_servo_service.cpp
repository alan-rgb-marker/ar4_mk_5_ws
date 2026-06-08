#include <chrono>
#include <memory>
#include <iostream>
#include <functional>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <moveit/utils/logger.hpp>
#include <std_msgs/msg/bool.hpp>

using std::placeholders::_1;

class moveit_servo_service : public rclcpp::Node
{
public:
    moveit_servo_service();
    ~moveit_servo_service() = default;

    // 建構子完成後才能呼叫，使用靜態工廠方法
    static std::shared_ptr<moveit_servo_service> create()
    {
        auto node = std::make_shared<moveit_servo_service>();
        node->init_moveit_servo(); // shared_ptr 已建立，安全呼叫
        return node;
    }

private:
    void init_moveit_servo();
    void moveit_servo_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void update_robot_state_callback(const std_msgs::msg::Bool msg);

    const std::string param_namespace = "moveit_servo";
    std::shared_ptr<const servo::ParamListener> servo_param_listener;
    servo::Params servo_params;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
    std::unique_ptr<moveit_servo::Servo> servo_;

    // joint_model_group 改為指標，延後初始化
    const moveit::core::JointModelGroup *joint_model_group = nullptr;

    std::deque<moveit_servo::KinematicState> joint_cmd_rolling_window;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr servo_twist_sub;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr update_robot_state_sub;

    moveit::core::RobotStatePtr robot_state_;

    rclcpp::WallRate rate;
};

moveit_servo_service::moveit_servo_service()
    : Node("moveit_servo_twist_control_node"),
      rate(1.0 / servo_params.publish_period)
{
    // 建構子只做最基本的事，不呼叫 shared_from_this()
}

void moveit_servo_service::init_moveit_servo()
{
    // 步驟 1：先建立 param_listener
    servo_param_listener =
        std::make_shared<const servo::ParamListener>(shared_from_this(), param_namespace);
    servo_params = servo_param_listener->get_params();

    // 步驟 2：建立 planning_scene_monitor
    planning_scene_monitor =
        moveit_servo::createPlanningSceneMonitor(shared_from_this(), servo_params);

    // 步驟 3：建立 Servo（依賴前兩者）
    servo_ = std::make_unique<moveit_servo::Servo>(
        shared_from_this(), servo_param_listener, planning_scene_monitor);

    servo_->setCommandType(moveit_servo::CommandType::TWIST);

    // 步驟 4：從 PlanningSceneMonitor 取得 robot_state，初始化 joint_model_group
    auto robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
    joint_model_group = robot_state->getJointModelGroup(servo_params.move_group_name);

    moveit_servo::KinematicState current_state = servo_->getCurrentRobotState(true);
    updateSlidingWindow(current_state, joint_cmd_rolling_window,
                        servo_params.max_expected_latency, this->now());

    // 步驟 5：建立 publisher 和 subscriber
    trajectory_outgoing_cmd_pub =
        create_publisher<trajectory_msgs::msg::JointTrajectory>(
            servo_params.command_out_topic, rclcpp::SystemDefaultsQoS());

    servo_twist_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "delta_twist_cmds", 10,
        std::bind(&moveit_servo_service::moveit_servo_callback, this, _1));

    update_robot_state_sub = this->create_subscription<std_msgs::msg::Bool>(
        "update_robot_state", 10,
        std::bind(&moveit_servo_service::update_robot_state_callback, this, _1));
}

void moveit_servo_service::moveit_servo_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{

    moveit_servo::TwistCommand twist_cmd = {
        msg->header.frame_id,
        {msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
         msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z}};

    moveit_servo::KinematicState joint_state =
        servo_->getNextJointState(this->robot_state_, twist_cmd);
    const moveit_servo::StatusCode status = servo_->getStatus();

    if (status != moveit_servo::StatusCode::INVALID)
    {
        updateSlidingWindow(joint_state, joint_cmd_rolling_window,
                            servo_params.max_expected_latency, this->now());

        if (const auto traj_msg = composeTrajectoryMessage(servo_params, joint_cmd_rolling_window))
        {
            trajectory_outgoing_cmd_pub->publish(traj_msg.value());
        }

        if (!joint_cmd_rolling_window.empty())
        {
            this->robot_state_->setJointGroupPositions(
                joint_model_group, joint_cmd_rolling_window.back().positions);
            this->robot_state_->setJointGroupVelocities(
                joint_model_group, joint_cmd_rolling_window.back().velocities);
        }
    }
    rate.sleep();
}

void moveit_servo_service::update_robot_state_callback(const std_msgs::msg::Bool msg)
{
    if (msg.data != true)
        return;

    this->robot_state_ = this->planning_scene_monitor->getStateMonitor()->getCurrentState();
    RCLCPP_INFO(this->get_logger(), "已經更新機器人狀態");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 使用工廠方法，確保 shared_ptr 建立後才呼叫 init
    auto node = moveit_servo_service::create();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}