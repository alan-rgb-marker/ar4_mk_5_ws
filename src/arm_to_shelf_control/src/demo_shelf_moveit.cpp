#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char **argv)
{
    // 1. 初始化 ROS2
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "simple_moveit2_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // 建立 Logger
    auto const logger = rclcpp::get_logger("demo_shelf_moveit_node");

    // 2. 建立 MoveIt MoveGroup 接口
    // 注意：請將 "ur_manipulator" 換成你的機器人定義名稱 (例如 "arm")
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    // 3. 設定目標位姿 (Target Pose)
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0; // 四元數 w=1 代表不旋轉
    target_pose.position.x = 0.367;
    target_pose.position.y = 0.000;
    target_pose.position.z = 0.287;
    target_pose.orientation.x = 0.704;
    target_pose.orientation.y = 0.704;
    target_pose.orientation.z = 0.062;
    target_pose.orientation.w = 0.062;

    move_group.setPoseTarget(target_pose);

    // 4. 進行路徑規劃
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(logger, "規劃成功，開始執行移動！");
        // 5. 執行移動
        move_group.execute(my_plan);
    }
    else
    {
        RCLCPP_ERROR(logger, "規劃失敗！");
    }

    // 6. 關閉
    rclcpp::shutdown();
    return 0;
}