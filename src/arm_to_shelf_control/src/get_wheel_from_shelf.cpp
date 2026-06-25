#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vision_interfaces/srv/armcoodinate.hpp>
#include <vision_interfaces/srv/shelf_coodinate.hpp>
#include <std_msgs/msg/bool.hpp>

#include <thread>
#include <memory>
#include <string>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

static const std::string ARM_GROUP = "arm";
static const std::string GRIPPER_GROUP = "gripper";

// -----------------------------------------------------------------------
// 放置目標高度（base_link frame，單位公尺）
// 當夾爪 TCP 的 Z 到達這個值時，Servo 停止
// -----------------------------------------------------------------------
static constexpr double LIFT_TARGET_Z = 0.35; // ← 依實際需求調整

// Servo 速度參數
static constexpr double SERVO_Y_VEL = 0.02; // 跟隨輪胎 Y 方向（會從 shelf_vel 覆蓋）
static constexpr double SERVO_Z_VEL = 0.03; // 向上提升速度 (m/s)

class ArmToWheelControl : public rclcpp::Node
{
public:
    ArmToWheelControl();
    ~ArmToWheelControl() = default;

    void moveit_init();

private:
    // ── ROS 介面 ──────────────────────────────────────────────────────
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
    rclcpp::Client<vision_interfaces::srv::Armcoodinate>::SharedPtr view_coord_cli_;
    rclcpp::Client<vision_interfaces::srv::ShelfCoodinate>::SharedPtr wheel_coord_cli_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_pub_;

    // ── MoveIt 介面 ───────────────────────────────────────────────────
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

    // ── 狀態資料 ──────────────────────────────────────────────────────
    double wheel_vel_y_ = 0.0;        // 輪胎 Y 軸速度 (m/s)
    rclcpp::Time wheel_feature_time_; // 輪胎座標對應的時間戳
    rclcpp::Duration elapsed_time_{0, 0};

    // ── 流水線步驟 ────────────────────────────────────────────────────
    void run();
    void run_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // 步驟 1：移到觀測位置
    void step1_move_to_observe();

    // 步驟 2：取得輪胎座標＋速度，預測未來位置，移過去
    void step2_move_to_wheel_future_pose();

    // 步驟 3：夾住輪胎
    void step3_grip_wheel();

    // 步驟 4：Servo 等速 Y 跟隨 + Z 向上，直到目標高度
    void step4_servo_lift();

    // ── 底層工具 ──────────────────────────────────────────────────────
    bool arm_planner(geometry_msgs::msg::Pose &target_pose,
                     const std::string &state = "normal",
                     double cli_used_time = 0.0);
    bool gripper_planner(const std::string &target = "close");
};

// =======================================================================
// main
// =======================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmToWheelControl>();
    node->moveit_init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// =======================================================================
// 建構子
// =======================================================================
ArmToWheelControl::ArmToWheelControl()
    : Node("arm_to_wheel_control"), elapsed_time_(0, 0)
{
    view_coord_cli_ = create_client<vision_interfaces::srv::Armcoodinate>("view_shelf_coord");
    wheel_coord_cli_ = create_client<vision_interfaces::srv::ShelfCoodinate>("shelf_coord");
    servo_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("delta_twist_cmds", 10);

    trigger_srv_ = create_service<std_srvs::srv::Trigger>(
        "run_wheel_service",
        std::bind(&ArmToWheelControl::run_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "ArmToWheelControl 節點已啟動，等待 trigger...");
}

void ArmToWheelControl::moveit_init()
{
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), ARM_GROUP);
    gripper_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), GRIPPER_GROUP);
}

// =======================================================================
// Trigger 服務回呼：把 run() 丟到背景執行緒，立刻回覆
// =======================================================================
void ArmToWheelControl::run_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    RCLCPP_INFO(get_logger(), "收到 trigger，啟動輪胎撿取流程...");
    std::thread([this]()
                { run(); })
        .detach();
    res->success = true;
    res->message = "Wheel pickup pipeline started.";
}

// =======================================================================
// 主流水線
// =======================================================================
void ArmToWheelControl::run()
{
    // ── 步驟 1 ────────────────────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "=== 步驟 1：移到觀測位置 ===");
    step1_move_to_observe();

    // 等待視覺節點穩定偵測
    std::this_thread::sleep_for(3s);

    // ── 步驟 2 ────────────────────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "=== 步驟 2：預測輪胎未來位置並移動 ===");
    step2_move_to_wheel_future_pose();

    // 等待到達輪胎特徵時間點
    RCLCPP_INFO(get_logger(), "等待輪胎特徵時間點...");
    while (now() < wheel_feature_time_)
    {
        std::this_thread::sleep_for(1ms);
    }
    RCLCPP_INFO(get_logger(), "時間到，手臂與輪胎位置同步！");

    // ── 步驟 3 ────────────────────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "=== 步驟 3：夾住輪胎 ===");
    step3_grip_wheel();

    // ── 步驟 4 ────────────────────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "=== 步驟 4：Servo 跟隨 Y 軸 + Z 軸向上提起 ===");
    step4_servo_lift();

    RCLCPP_INFO(get_logger(), "=== 流水線完成 ===");
}

// =======================================================================
// 步驟 1：移到觀測位置
//   呼叫 view_shelf_coord 服務，取得觀測點 pose，執行規劃
// =======================================================================
void ArmToWheelControl::step1_move_to_observe()
{
    // 等待服務上線
    while (!view_coord_cli_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(get_logger(), "等待 view_shelf_coord 時被中斷");
            return;
        }
        RCLCPP_WARN(get_logger(), "view_shelf_coord 服務尚未上線，繼續等待...");
    }

    auto req = std::make_shared<vision_interfaces::srv::Armcoodinate::Request>();
    req->result = "get_view_shelf_coord"; // ← 沿用原本 key，或改成 "get_view_wheel_coord"

    auto future = view_coord_cli_->async_send_request(req);
    auto res = future.get();

    arm_planner(res->arm_cood);
    RCLCPP_INFO(get_logger(), "步驟 1 完成：已到達觀測位置");
}

// =======================================================================
// 步驟 2：取得輪胎座標＋速度 → 預測未來 Y 位置 → 移動手臂
//
//   與原本 move_to_shelf_pose() 邏輯相同，
//   只是把 shelf 語意換成 wheel，並移除 X/Z 額外修正（輪胎只在 Y 移動）
// =======================================================================
void ArmToWheelControl::step2_move_to_wheel_future_pose()
{
    while (!wheel_coord_cli_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(get_logger(), "等待 shelf_coord 時被中斷");
            return;
        }
        RCLCPP_WARN(get_logger(), "shelf_coord 服務尚未上線，繼續等待...");
    }

    auto req = std::make_shared<vision_interfaces::srv::ShelfCoodinate::Request>();
    req->req_cmd = "get_wheel_from_shelf_coord";

    auto future = wheel_coord_cli_->async_send_request(req);
    auto res = future.get();

    // 記錄視覺特徵對應的時間戳與速度
    wheel_feature_time_ = rclcpp::Time(res->start_pos_time);
    wheel_vel_y_ = res->shelf_vel; // 輪胎 Y 軸速度 (m/s)，正負代表方向

    geometry_msgs::msg::Pose target_pose = res->shelf_pose;

    // 記錄服務呼叫耗時（用來補償延遲）
    rclcpp::Time end_cli = now();
    elapsed_time_ = end_cli - wheel_feature_time_;

    RCLCPP_INFO(get_logger(), "輪胎速度 Y: %.4f m/s，服務耗時: %.3f s",
                wheel_vel_y_, elapsed_time_.seconds());

    // arm_planner 裡的 "feature_postion" 模式會自動計算規劃時間 + 執行時間，
    // 再把 target_pose.position.y 往前推，讓手臂到達時恰好與輪胎同步
    arm_planner(target_pose, "feature_postion", elapsed_time_.seconds());

    RCLCPP_INFO(get_logger(), "步驟 2 完成：已移動到輪胎預測位置");
}

// =======================================================================
// 步驟 3：夾住輪胎
// =======================================================================
void ArmToWheelControl::step3_grip_wheel()
{
    bool ok = gripper_planner("close");
    if (ok)
        RCLCPP_INFO(get_logger(), "步驟 3 完成：輪胎已夾緊");
    else
        RCLCPP_ERROR(get_logger(), "步驟 3 失敗：夾爪規劃錯誤");
}

// =======================================================================
// 步驟 4：Servo 等速 Y 跟隨 + Z 向上，直到 TCP Z >= LIFT_TARGET_Z
//
//   twist 設定：
//     linear.y = wheel_vel_y_   （跟隨輪胎，避免側向拉扯）
//     linear.z = SERVO_Z_VEL    （向上提起）
//   結束條件：current TCP Z >= LIFT_TARGET_Z
// =======================================================================
void ArmToWheelControl::step4_servo_lift()
{
    geometry_msgs::msg::TwistStamped twist;
    twist.header.frame_id = "base_link";
    twist.twist.linear.x = 0.0;
    twist.twist.linear.y = wheel_vel_y_; // 跟隨輪胎
    twist.twist.linear.z = SERVO_Z_VEL;  // 向上
    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = 0.0;

    constexpr double PUBLISH_RATE_HZ = 50.0;
    rclcpp::Rate rate(PUBLISH_RATE_HZ);

    RCLCPP_INFO(get_logger(), "開始 Servo 提升，目標 Z: %.3f m", LIFT_TARGET_Z);

    while (rclcpp::ok())
    {
        geometry_msgs::msg::PoseStamped current = move_group_->getCurrentPose("gripper_tcp");
        double current_z = current.pose.position.z;

        if (current_z >= LIFT_TARGET_Z)
        {
            RCLCPP_INFO(get_logger(), "已到達目標高度 Z=%.3f，停止 Servo", current_z);
            break;
        }

        // 接近目標時減速，避免超衝（進入最後 2 cm 時減半速）
        double remaining = LIFT_TARGET_Z - current_z;
        if (remaining < 0.02)
            twist.twist.linear.z = SERVO_Z_VEL * 0.5;
        else
            twist.twist.linear.z = SERVO_Z_VEL;

        twist.header.stamp = now();
        servo_pub_->publish(twist);
        rate.sleep();
    }

    // 發一個全零 twist 讓 Servo 停止
    geometry_msgs::msg::TwistStamped stop_twist;
    stop_twist.header.frame_id = "base_link";
    stop_twist.header.stamp = now();
    servo_pub_->publish(stop_twist);

    RCLCPP_INFO(get_logger(), "步驟 4 完成：輪胎已提升到目標高度");
}

// =======================================================================
// arm_planner：與原版相同，保留 "normal" 和 "feature_postion" 兩種模式
// =======================================================================
bool ArmToWheelControl::arm_planner(geometry_msgs::msg::Pose &target_pose,
                                    const std::string &state,
                                    double cli_used_time)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (state == "normal")
    {
        move_group_->setPoseTarget(target_pose);
        bool ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!ok)
        {
            RCLCPP_ERROR(get_logger(), "手臂規劃失敗（normal）");
            return false;
        }
    }
    else if (state == "feature_postion")
    {
        // ── 第一次規劃：取得預估執行時間 ──────────────────────────────
        move_group_->setPoseTarget(target_pose);
        bool ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!ok)
        {
            RCLCPP_WARN(get_logger(), "第一次規劃失敗");
        }

        double plan_time = plan.planning_time;
        double move_time = 0.0;
        if (!plan.trajectory.joint_trajectory.points.empty())
        {
            auto &last = plan.trajectory.joint_trajectory.points.back();
            move_time = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9;
            RCLCPP_INFO(get_logger(), "預估執行時間: %.3f s", move_time);
        }

        // ── 計算總偏移時間，推算輪胎 Y 位移 ──────────────────────────
        constexpr double SAFETY_OFFSET = 0.5; // 緩衝秒數
        double t = plan_time * 2.0 + move_time + std::abs(cli_used_time) + SAFETY_OFFSET;
        RCLCPP_INFO(get_logger(), "總補償時間 t=%.3f s", t);

        double dy = wheel_vel_y_ * t; // 輪胎在 t 秒後的 Y 位移
        target_pose.position.y += dy;
        wheel_feature_time_ += rclcpp::Duration::from_seconds(t);

        RCLCPP_INFO(get_logger(), "目標 Y 補償後: %.4f m（補償 dy=%.4f m）",
                    target_pose.position.y, dy);

        // ── 第二次規劃：用補償後的座標重新規劃 ───────────────────────
        move_group_->setPoseTarget(target_pose);
        ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!ok)
        {
            RCLCPP_ERROR(get_logger(), "第二次規劃失敗");
            return false;
        }
    }

    RCLCPP_INFO(get_logger(), "規劃成功，開始執行...");
    move_group_->execute(plan);
    return true;
}

// =======================================================================
// gripper_planner
// =======================================================================
bool ArmToWheelControl::gripper_planner(const std::string &target)
{
    gripper_group_->setNamedTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (gripper_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok)
    {
        RCLCPP_ERROR(get_logger(), "夾爪規劃失敗（%s）", target.c_str());
        return false;
    }

    gripper_group_->execute(plan);
    return true;
}