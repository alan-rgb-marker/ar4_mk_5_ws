#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <thread>
#include <chrono>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "moveit_xyz_control",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // executor
  auto executor =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor->add_node(node);

  std::thread spinner([&executor]()
                      { executor->spin(); });

  // MoveGroup
  static const std::string ARM_GROUP = "arm";
  static const std::string GRIPPER_GROUP = "gripper";

  moveit::planning_interface::MoveGroupInterface move_group(node, ARM_GROUP);

  moveit::planning_interface::MoveGroupInterface gripper_group(node, GRIPPER_GROUP);

  rclcpp::sleep_for(std::chrono::seconds(2));

  // 顯示資訊
  RCLCPP_INFO(node->get_logger(),
              "Planning Frame: %s",
              move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(node->get_logger(),
              "End Effector Link: %s",
              move_group.getEndEffectorLink().c_str());

  // =====================================================
  // 固定 XYZ 座標
  // =====================================================

  geometry_msgs::msg::PoseStamped target_pose;

  // 指定座標系
  target_pose.header.frame_id = "base_link";

  // 目標位置
  target_pose.pose.position.x = 0.3;
  target_pose.pose.position.y = 0.0;
  target_pose.pose.position.z = 0.1562; // 0.289;

  // 姿態 Quaternion
  target_pose.pose.orientation.x = 0.707;
  target_pose.pose.orientation.y = 0.707;
  target_pose.pose.orientation.z = 0.0;
  target_pose.pose.orientation.w = -0.0;

  // 設定目標
  move_group.setPoseTarget(target_pose);

  // 規劃
  moveit::planning_interface::MoveGroupInterface::Plan arm_plan;

  bool success =
      (move_group.plan(arm_plan) ==
       moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(node->get_logger(),
                "手臂規劃成功，開始執行");

    move_group.execute(arm_plan);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(),
                 "手臂規劃失敗");
  }

  // =====================================================
  // 夾爪打開
  // =====================================================

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(node->get_logger(),
              "執行夾爪 open");

  gripper_group.setNamedTarget("open");

  moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

  bool gripper_success =
      (gripper_group.plan(gripper_plan) ==
       moveit::core::MoveItErrorCode::SUCCESS);

  if (gripper_success)
  {
    RCLCPP_INFO(node->get_logger(),
                "夾爪規劃成功");

    gripper_group.execute(gripper_plan);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(),
                 "夾爪規劃失敗");
  }

  // =====================================================
  // 結束
  // =====================================================

  executor->cancel();

  if (spinner.joinable())
  {
    spinner.join();
  }

  rclcpp::shutdown();

  return 0;
}
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose.hpp>
// #include <thread>
// #include <chrono>

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);

//   auto node = std::make_shared<rclcpp::Node>(
//       "hello_moveit",
//       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

//   auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
//   executor->add_node(node);
//   auto thread = std::thread([executor]()
//                             { executor->spin(); });

//   // --- 手動控制部分 ---
//   static const std::string PLANNING_GROUP = "arm";
//   moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
//   moveit::planning_interface::MoveGroupInterface gripper_group(node, "gripper");

//   rclcpp::sleep_for(std::chrono::seconds(2));

//   // 1. 手臂移動
//   auto current_pose = move_group.getCurrentPose().pose;
//   geometry_msgs::msg::Pose target_pose = current_pose;
//   target_pose.position.z += 0.1;

//   move_group.setPoseTarget(target_pose);
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   if (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS)
//   {
//     RCLCPP_INFO(node->get_logger(), "手臂規劃成功，開始移動...");
//     move_group.execute(my_plan);
//   }

//   // 2. 夾爪打開 (Named Target)
//   // 在手臂移動一段距離後，執行夾爪動作
//   RCLCPP_INFO(node->get_logger(), "正在執行夾爪動作：open");
//   gripper_group.setNamedTarget("open"); // 使用 SRDF 裡的名稱
//   gripper_group.move();                 // 或者用 plan() + execute()

//   auto gripper_name = gripper_group.getNamedTargets();
//   RCLCPP_INFO(node->get_logger(), "夾爪名稱: %s", gripper_name.front().c_str());

//   moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
//   if (gripper_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS)
//   {
//     RCLCPP_INFO(node->get_logger(), "夾爪規劃成功，開始執行...");
//     gripper_group.execute(gripper_plan);
//   }
//   else
//   {
//     RCLCPP_ERROR(node->get_logger(), "夾爪規劃失敗！");
//   }

//   executor->cancel();
//   if (thread.joinable())
//     thread.join();
//   rclcpp::shutdown();
//   return 0;
// }
