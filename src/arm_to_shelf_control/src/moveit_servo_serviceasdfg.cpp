#include <rclcpp/rclcpp.hpp>
#include <moveit_servo/servo.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_servo/utils/common.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

using namespace moveit_servo;

class ServoTwistNode : public rclcpp::Node
{
public:
    // 建構子只做「不需要 shared_from_this()」的事
    ServoTwistNode() : Node("servo_twist_node") {}

    // 真正的初始化放在這裡，由 main() 在 make_shared 之後呼叫
    void initialize()
    {
        RCLCPP_INFO(get_logger(), "Starting MoveIt Servo Twist Node for Jazzy");

        // PSM：Jazzy 簽名是 (node, robot_description_param_name, monitor_name)
        psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            shared_from_this(), "robot_description", "servo_planning_scene_monitor");

        psm_->startStateMonitor("/joint_states");
        psm_->startSceneMonitor();
        psm_->startPublishingPlanningScene(
            planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
        psm_->setPlanningScenePublishingFrequency(25.0);

        servo_param_listener_ = std::make_shared<const servo::ParamListener>(
            shared_from_this(), "moveit_servo");

        servo_ = std::make_unique<Servo>(
            shared_from_this(), servo_param_listener_, psm_);

        servo_->setCommandType(CommandType::TWIST);

        twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "delta_twist_cmds", rclcpp::QoS(10),
            std::bind(&ServoTwistNode::twistCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Servo ready.");
    }

private:
    void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        if (rclcpp::Time(msg->header.stamp) < now() - rclcpp::Duration::from_seconds(0.2))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Stale Twist command ignored");
            return;
        }

        TwistCommand command;
        command.frame_id = msg->header.frame_id.empty()
                               ? "base_link"
                               : msg->header.frame_id;
        command.velocities << msg->twist.linear.x,
            msg->twist.linear.y,
            msg->twist.linear.z,
            msg->twist.angular.x,
            msg->twist.angular.y,
            msg->twist.angular.z;

        auto current_state = psm_->getStateMonitor()->getCurrentState();
        if (!current_state)
        {
            RCLCPP_WARN(get_logger(), "Cannot get current robot state!");
            return;
        }

        KinematicState next_state = servo_->getNextJointState(current_state, command);

        // TODO: publish next_state 到 /joint_trajectory_controller/joint_trajectory
        RCLCPP_INFO_ONCE(get_logger(), "Successfully processed first Twist command");
    }

    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
    std::shared_ptr<const servo::ParamListener> servo_param_listener_;
    std::unique_ptr<Servo> servo_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoTwistNode>();
    node->initialize(); // shared_ptr 已建立，shared_from_this() 安全
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
// /*******************************************************************************
//  * servo_twist_controller.cpp
//  *
//  * 單一 class ServoTwistController，繼承 rclcpp::Node
//  * 訂閱 geometry_msgs/msg/TwistStamped 控制機器人：
//  *   - 收到非零 twist  → 持續發送該 twist
//  *   - 收到全零 twist  → 停止
//  *   - 沒有新訊息      → 維持當前狀態（動繼續動，停繼續停）
//  *
//  * 兩段式初始化（two-phase init）：
//  *   constructor 只做不需要 shared_from_this() 的事，
//  *   需要 shared_ptr 已建立的邏輯全部放進 initialize()，
//  *   在 main 裡 make_shared 之後再呼叫。
//  *******************************************************************************/

// #include <chrono>
// #include <deque>

// #include <geometry_msgs/msg/twist_stamped.hpp>
// #include <moveit_servo/servo.hpp>
// #include <moveit_servo/utils/common.hpp>
// #include <moveit/utils/logger.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <trajectory_msgs/msg/joint_trajectory.hpp>

// #include <std_msgs/msg/bool.hpp>

// using namespace moveit_servo;

// class ServoTwistController : public rclcpp::Node
// {
// public:
//     // ── constructor：只做「不需要 shared_from_this()」的初始化 ────────
//     ServoTwistController()
//         : Node("servo_twist_controller"), is_moving_(false)
//     {
//         moveit::setNodeLoggerName(this->get_name());

//         // 預設 twist：全零（靜止）
//         current_twist_.header.frame_id = "base_link";
//         current_twist_.twist = geometry_msgs::msg::Twist{};
//     }

//     // ── initialize()：需要 shared_from_this() 的邏輯，在 main 裡呼叫 ──
//     void initialize()
//     {
//         // ── Servo 初始化 ──────────────────────────────────────────────
//         const std::string param_namespace = "moveit_servo";
//         servo_param_listener_ =
//             std::make_shared<const servo::ParamListener>(shared_from_this(), param_namespace);
//         servo_params_ = servo_param_listener_->get_params();

//         planning_scene_monitor_ = createPlanningSceneMonitor(shared_from_this(), servo_params_);
//         servo_ = std::make_unique<Servo>(shared_from_this(), servo_param_listener_, planning_scene_monitor_);
//         servo_->setCommandType(CommandType::TWIST);

//         // ── Publisher：把 JointTrajectory 送給 controller ────────────
//         trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
//             servo_params_.command_out_topic, rclcpp::SystemDefaultsQoS());

//         // ── Subscriber：接收外部 TwistStamped 指令 ───────────────────
//         twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
//             "servo_twist_cmd",
//             rclcpp::SystemDefaultsQoS(),
//             [this](geometry_msgs::msg::TwistStamped::SharedPtr msg)
//             {
//                 twistCallback(msg);
//             });

//         // ── 初始化 rolling window ─────────────────────────────────────
//         robot_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
//         joint_model_group_ = robot_state_->getJointModelGroup(servo_params_.move_group_name);

//         KinematicState current_state = servo_->getCurrentRobotState(true /* wait for updated state */);
//         updateSlidingWindow(current_state, joint_cmd_rolling_window_,
//                             servo_params_.max_expected_latency, this->now());

//         // ── 控制迴圈 Timer（放最後，確保其他成員都已就緒）────────────
//         const auto period = std::chrono::duration<double>(servo_params_.publish_period);
//         control_timer_ = this->create_wall_timer(
//             std::chrono::duration_cast<std::chrono::nanoseconds>(period),
//             [this]()
//             { controlLoop(); });

//         // reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(
//         //     "reset_servo_cmd",
//         //     rclcpp::SystemDefaultsQoS(),
//         //     [this](std_msgs::msg::Bool::SharedPtr msg)
//         //     {
//         //         if (msg->data) // 如果收到 true
//         //         {
//         //             // 🌟 瞬間清空窗口
//         //             joint_cmd_rolling_window_.clear();
//         //         }
//         //     });

//         RCLCPP_INFO(this->get_logger(), "ServoTwistController 已啟動，等待指令...");
//         RCLCPP_INFO_STREAM(this->get_logger(), servo_->getStatusMessage());
//     }

// private:
//     // ── Callback：收到新的 TwistStamped ──────────────────────────────
//     void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
//     {
//         current_twist_ = *msg;

//         if (isTwistZero(msg->twist))
//         {
//             if (is_moving_)
//             {
//                 RCLCPP_INFO(this->get_logger(), "收到零 twist，停止移動");
//                 is_moving_ = false;
//             }
//         }
//         else
//         {
//             if (!is_moving_)
//             {
//                 RCLCPP_INFO(this->get_logger(), "收到非零 twist，開始移動");
//                 is_moving_ = true;
//             }
//         }
//     }

//     // ── 控制迴圈：由 Timer 週期性呼叫 ────────────────────────────────
//     void controlLoop()
//     {
//         // 靜止狀態不發指令
//         if (!is_moving_)
//             // this->joint_cmd_rolling_window_.clear();
//             return;

//         // 把 TwistStamped 轉成 moveit_servo 的 TwistCommand
//         TwistCommand twist_cmd{
//             current_twist_.header.frame_id,
//             {current_twist_.twist.linear.x,
//              current_twist_.twist.linear.y,
//              current_twist_.twist.linear.z,
//              current_twist_.twist.angular.x,
//              current_twist_.twist.angular.y,
//              current_twist_.twist.angular.z}};
//         this->robot_state_ = this->planning_scene_monitor_->getStateMonitor()->getCurrentState();

//         KinematicState joint_state = servo_->getNextJointState(robot_state_, twist_cmd);
//         const StatusCode status = servo_->getStatus();

//         if (status != StatusCode::INVALID)
//         {
//             updateSlidingWindow(joint_state, joint_cmd_rolling_window_,
//                                 servo_params_.max_expected_latency, this->now());

//             if (const auto msg = composeTrajectoryMessage(servo_params_, joint_cmd_rolling_window_))
//             {
//                 trajectory_pub_->publish(msg.value());
//             }

//             // 更新 robot_state 供下一個迴圈使用
//             if (!joint_cmd_rolling_window_.empty())
//             {
//                 robot_state_->setJointGroupPositions(joint_model_group_,
//                                                      joint_cmd_rolling_window_.back().positions);
//                 robot_state_->setJointGroupVelocities(joint_model_group_,
//                                                       joint_cmd_rolling_window_.back().velocities);
//             }
//         }
//         else
//         {
//             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//                                  "Servo 狀態 INVALID，跳過本次指令: %s",
//                                  servo_->getStatusMessage().c_str());
//         }
//     }

//     // ── 工具：判斷 twist 是否全零 ─────────────────────────────────────
//     static bool isTwistZero(const geometry_msgs::msg::Twist &twist)
//     {
//         constexpr double EPS = 1e-6;
//         return std::abs(twist.linear.x) < EPS &&
//                std::abs(twist.linear.y) < EPS &&
//                std::abs(twist.linear.z) < EPS &&
//                std::abs(twist.angular.x) < EPS &&
//                std::abs(twist.angular.y) < EPS &&
//                std::abs(twist.angular.z) < EPS;
//     }

//     // ── 成員變數 ──────────────────────────────────────────────────────
//     std::shared_ptr<const servo::ParamListener> servo_param_listener_;
//     servo::Params servo_params_;
//     planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
//     std::unique_ptr<Servo> servo_;

//     rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
//     rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
//     rclcpp::TimerBase::SharedPtr control_timer_;

//     moveit::core::RobotStatePtr robot_state_;
//     const moveit::core::JointModelGroup *joint_model_group_;
//     std::deque<KinematicState> joint_cmd_rolling_window_;

//     geometry_msgs::msg::TwistStamped current_twist_;
//     bool is_moving_;

//     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
//     std::mutex mutex_;
// };

// // ── main ──────────────────────────────────────────────────────────────────────
// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     // Step 1：make_shared → shared_ptr 建立，weak_ptr 可用
//     auto node = std::make_shared<ServoTwistController>();

//     // Step 2：這時候 shared_from_this() 已經安全，才呼叫 initialize()
//     node->initialize();

//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }