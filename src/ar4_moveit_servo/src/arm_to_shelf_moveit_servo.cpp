/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, PickNik Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title     : demo_twist.cpp
 *      Project   : moveit_servo
 *      Created   : 06/01/2023
 *      Author    : V Mohammed Ibrahim
 *
 *      Description : Example of controlling a robot through twist commands via the C++ API.
 */

#include <chrono>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// For Rolling, Kilted, and newer
#if RCLCPP_VERSION_GTE(29, 6, 0)
#include <tf2_ros/transform_listener.hpp>
// For Jazzy and older
#else
#include <tf2_ros/transform_listener.h>
#endif
#include <moveit/utils/logger.hpp>

using namespace moveit_servo;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // The servo object expects to get a ROS node.
    const rclcpp::Node::SharedPtr demo_node = std::make_shared<rclcpp::Node>("moveit_servo_demo");
    moveit::setNodeLoggerName(demo_node->get_name());

    // Get the servo parameters.
    const std::string param_namespace = "moveit_servo";
    const std::shared_ptr<const servo::ParamListener> servo_param_listener =
        std::make_shared<const servo::ParamListener>(demo_node, param_namespace);
    const servo::Params servo_params = servo_param_listener->get_params();

    // The publisher to send trajectory message to the robot controller.
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub =
        demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(servo_params.command_out_topic,
                                                                           rclcpp::SystemDefaultsQoS());

    // Create the servo object
    const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
        createPlanningSceneMonitor(demo_node, servo_params);
    Servo servo = Servo(demo_node, servo_param_listener, planning_scene_monitor);

    // Wait for some time, so that the planning scene is loaded in rviz.
    // This is just for convenience, should not be used for sync in real application.
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Get the robot state and joint model group info.
    auto robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
    const moveit::core::JointModelGroup *joint_model_group =
        robot_state->getJointModelGroup(servo_params.move_group_name);

    // Set the command type for servo.
    servo.setCommandType(CommandType::TWIST);

    // Move end effector in the +z direction at 5 cm/s
    // while turning around z axis in the +ve direction at 0.4 rad/s
    TwistCommand target_twist{"base_link", {0.0, 0.01, 0.0, 0.0, 0.0, 0.0}};

    // Frequency at which commands will be sent to the robot controller.
    rclcpp::WallRate rate(1.0 / servo_params.publish_period);

    std::chrono::seconds timeout_duration(4);
    std::chrono::seconds time_elapsed(0);
    const auto start_time = std::chrono::steady_clock::now();

    // create command queue to build trajectory message and add current robot state
    std::deque<KinematicState> joint_cmd_rolling_window;
    KinematicState current_state = servo.getCurrentRobotState(true /* wait for updated state */);
    updateSlidingWindow(current_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());

    RCLCPP_INFO_STREAM(demo_node->get_logger(), servo.getStatusMessage());
    while (rclcpp::ok())
    {
        KinematicState joint_state = servo.getNextJointState(robot_state, target_twist);
        const StatusCode status = servo.getStatus();

        const auto current_time = std::chrono::steady_clock::now();
        time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
        if (time_elapsed > timeout_duration)
        {
            RCLCPP_INFO_STREAM(demo_node->get_logger(), "Timed out");
            break;
        }
        else if (status != StatusCode::INVALID)
        {
            updateSlidingWindow(joint_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());
            if (const auto msg = composeTrajectoryMessage(servo_params, joint_cmd_rolling_window))
            {
                trajectory_outgoing_cmd_pub->publish(msg.value());
            }
            if (!joint_cmd_rolling_window.empty())
            {
                robot_state->setJointGroupPositions(joint_model_group, joint_cmd_rolling_window.back().positions);
                robot_state->setJointGroupVelocities(joint_model_group, joint_cmd_rolling_window.back().velocities);
            }
        }
        rate.sleep();
    }

    RCLCPP_INFO(demo_node->get_logger(), "Exiting demo.");
    rclcpp::shutdown();
}

// #include <atomic>
// #include <chrono>
// #include <moveit_servo/servo.hpp>
// #include <moveit_servo/utils/common.hpp>
// #include <mutex>
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp/version.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// #include <memory>
// #include <string>

// // For Rolling, Kilted, and newer
// #if RCLCPP_VERSION_GTE(29, 6, 0)
// #include <tf2_ros/transform_listener.hpp>
// // For Jazzy and older
// #else
// #include <tf2_ros/transform_listener.h>
// #endif
// #include <moveit/utils/logger.hpp>

// using namespace moveit_servo;

// namespace
// {
//     constexpr auto K_BASE_FRAME = "base_link";
//     constexpr auto K_TIP_FRAME = "gripper_tcp";
// } // namespace

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     // The servo object expects to get a ROS node.
//     const rclcpp::Node::SharedPtr demo_node = std::make_shared<rclcpp::Node>("my_servo_node");
//     moveit::setNodeLoggerName(demo_node->get_name());

//     // Get the servo parameters.
//     const std::string param_namespace = "moveit_servo";
//     const std::shared_ptr<const servo::ParamListener> servo_param_listener =
//         std::make_shared<const servo::ParamListener>(demo_node, param_namespace);
//     const servo::Params servo_params = servo_param_listener->get_params();

//     // The publisher to send trajectory message to the robot controller.
//     rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub =
//         demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(servo_params.command_out_topic, rclcpp::SystemDefaultsQoS());

//     // Create the servo object
//     const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
//         createPlanningSceneMonitor(demo_node, servo_params);
//     Servo servo = Servo(demo_node, servo_param_listener, planning_scene_monitor);

//     // Helper function to get the current pose of a specified frame.
//     const auto get_current_pose = [](const std::string &target_frame, const moveit::core::RobotStatePtr &robot_state)
//     {
//         return robot_state->getGlobalLinkTransform(target_frame);
//     };

//     // Wait for some time, so that the planning scene is loaded in rviz.
//     // This is just for convenience, should not be used for sync in real application.
//     std::this_thread::sleep_for(std::chrono::seconds(3));

//     // Get the robot state and joint model group info.
//     auto robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
//     const moveit::core::JointModelGroup *joint_model_group =
//         robot_state->getJointModelGroup(servo_params.move_group_name);

//     // Set the command type for servo.
//     servo.setCommandType(CommandType::POSE);

//     // The dynamically updated target pose.
//     // PoseCommand target_pose;
//     // target_pose.frame_id = K_BASE_FRAME;

//     // // Initializing the target pose as end effector pose, this can be any pose.
//     // target_pose.pose = get_current_pose(K_TIP_FRAME, robot_state);

//     // // servo loop will exit upon reaching this pose.
//     // Eigen::Isometry3d terminal_pose = target_pose.pose;
//     // terminal_pose.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
//     // terminal_pose.translate(Eigen::Vector3d(0.0, 0.0, -0.1));

//     // // The target pose (frame being tracked) moves by this step size each iteration.
//     // Eigen::Vector3d linear_step_size{0.0, 0.0, -0.001};
//     // Eigen::AngleAxisd angular_step_size(0.01, Eigen::Vector3d::UnitZ());

//     PoseCommand target_pose;
//     target_pose.frame_id = K_BASE_FRAME; // 確保以 base_link 為基準

//     // ==========================================
//     // 📍 1. 在這裡輸入你的自訂座標 (XYZ) 與姿態 (WXYZ)
//     // ==========================================
//     double target_x = 0.636; // 替換成你的 X
//     double target_y = 0.1; // 替換成你的 Y
//     double target_z = 0.138; // 替換成你的 Z

//     double target_qw = 0.354; // 替換成你的 W
//     double target_qx = 0.612; // 替換成你的 X
//     double target_qy = 0.612; // 替換成你的 Y
//     double target_qz = 0.354; // 替換成你的 Z

//     // 📍 2. 轉換為 Eigen::Isometry3d 格式
//     Eigen::Isometry3d my_custom_pose = Eigen::Isometry3d::Identity();
//     my_custom_pose.translate(Eigen::Vector3d(target_x, target_y, target_z));
//     my_custom_pose.rotate(Eigen::Quaterniond(target_qw, target_qx, target_qy, target_qz));

//     // 📍 3. 設定終點 (terminal_pose) 為你的自訂座標
//     Eigen::Isometry3d terminal_pose = my_custom_pose;

//     // 讓 Servo 一開始的目標先對準目前位置 (避免剛啟動時暴衝)
//     target_pose.pose = get_current_pose(K_TIP_FRAME, robot_state);

//     RCLCPP_INFO_STREAM(demo_node->get_logger(), servo.getStatusMessage());

//     rclcpp::WallRate servo_rate(1 / servo_params.publish_period);

//     // create command queue to build trajectory message and add current robot state
//     std::deque<KinematicState> joint_cmd_rolling_window;
//     KinematicState current_state = servo.getCurrentRobotState(true /* wait for updated state */);
//     updateSlidingWindow(current_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());

//     bool satisfies_linear_tolerance = false;
//     bool satisfies_angular_tolerance = false;
//     bool stop_tracking = false;
//     while (!stop_tracking && rclcpp::ok())
//     {
//         // // check if goal reached
//         // target_pose.pose = get_current_pose(K_TIP_FRAME, robot_state);
//         // satisfies_linear_tolerance |= target_pose.pose.translation().isApprox(terminal_pose.translation(), servo_params.pose_tracking.linear_tolerance);
//         // satisfies_angular_tolerance |=
//         //     target_pose.pose.rotation().isApprox(terminal_pose.rotation(), servo_params.pose_tracking.angular_tolerance);
//         // stop_tracking = satisfies_linear_tolerance && satisfies_angular_tolerance;

//         // // Dynamically update the target pose.
//         // // if (!satisfies_linear_tolerance)
//         // // {
//         // //     target_pose.pose.translate(linear_step_size);
//         // // }
//         // // if (!satisfies_angular_tolerance)
//         // // {
//         // //     target_pose.pose.rotate(angular_step_size);
//         // // }

//         // // 直接將最終目標座標賦予 Servo，讓 Servo 負責朝向該點計算運動學
//         // target_pose.pose = terminal_pose;

//         // // get next servo command
//         // KinematicState joint_state = servo.getNextJointState(robot_state, target_pose);
//         // StatusCode status = servo.getStatus();
//         // if (status != StatusCode::INVALID)
//         // {
//         //     updateSlidingWindow(joint_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());
//         //     if (const auto msg = composeTrajectoryMessage(servo_params, joint_cmd_rolling_window))
//         //     {
//         //         trajectory_outgoing_cmd_pub->publish(msg.value());
//         //     }
//         //     if (!joint_cmd_rolling_window.empty())
//         //     {
//         //         robot_state->setJointGroupPositions(joint_model_group, joint_cmd_rolling_window.back().positions);
//         //         robot_state->setJointGroupVelocities(joint_model_group, joint_cmd_rolling_window.back().velocities);
//         //     }
//         // }

//         // servo_rate.sleep();

//         Eigen::Isometry3d current_pose = get_current_pose(K_TIP_FRAME, robot_state);

//         // 2. 檢查是否到達終點 (拿當前真實位置與終點做比較)
//         satisfies_linear_tolerance = current_pose.translation().isApprox(
//             terminal_pose.translation(), servo_params.pose_tracking.linear_tolerance);

//         satisfies_angular_tolerance = current_pose.rotation().isApprox(
//             terminal_pose.rotation(), servo_params.pose_tracking.angular_tolerance);

//         stop_tracking = satisfies_linear_tolerance && satisfies_angular_tolerance;

//         if (stop_tracking)
//         {
//             RCLCPP_INFO(demo_node->get_logger(), "成功抵達自訂目標點！");
//             break;
//         }

//         // 3. 🧠 動態微步進（插值）控制
//         Eigen::Isometry3d next_step_pose = Eigen::Isometry3d::Identity();

//         // --- 【位置微步進】 ---
//         Eigen::Vector3d current_trans = current_pose.translation();
//         Eigen::Vector3d terminal_trans = terminal_pose.translation();
//         Eigen::Vector3d diff_trans = terminal_trans - current_trans;

//         // 限制每一步最多只走 1mm (0.001m)，可根據你的 publish_period 調整速度
//         double max_linear_step = 0.005;
//         if (diff_trans.norm() > max_linear_step)
//         {
//             next_step_pose.translation() = current_trans + diff_trans.normalized() * max_linear_step;
//         }
//         else
//         {
//             next_step_pose.translation() = terminal_trans;
//         }

//         // --- 【姿態微步進 (使用球面線性插值 SLERP)】 ---
//         Eigen::Quaterniond q_current(current_pose.rotation());
//         Eigen::Quaterniond q_terminal(terminal_pose.rotation());
//         double angle_diff = q_current.angularDistance(q_terminal);

//         // 限制每一步最多只旋轉 0.01 弧度 (約 0.57 度)
//         double max_angular_step = 0.01;
//         if (angle_diff > max_angular_step)
//         {
//             double t = max_angular_step / angle_diff;
//             next_step_pose.linear() = q_current.slerp(t, q_terminal).toRotationMatrix();
//         }
//         else
//         {
//             next_step_pose.linear() = q_terminal.toRotationMatrix();
//         }

//         // 4. 將這充滿安全感的「微步目標」餵給 Servo
//         target_pose.pose = next_step_pose;

//         // get next servo command
//         KinematicState joint_state = servo.getNextJointState(robot_state, target_pose);
//         StatusCode status = servo.getStatus();
//         if (status != StatusCode::INVALID)
//         {
//             updateSlidingWindow(joint_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());
//             if (const auto msg = composeTrajectoryMessage(servo_params, joint_cmd_rolling_window))
//             {
//                 trajectory_outgoing_cmd_pub->publish(msg.value());
//             }
//             if (!joint_cmd_rolling_window.empty())
//             {
//                 robot_state->setJointGroupPositions(joint_model_group, joint_cmd_rolling_window.back().positions);
//                 robot_state->setJointGroupVelocities(joint_model_group, joint_cmd_rolling_window.back().velocities);
//             }
//         }

//         servo_rate.sleep();
//     }

//     RCLCPP_INFO_STREAM(demo_node->get_logger(), "REACHED : " << stop_tracking);
//     RCLCPP_INFO(demo_node->get_logger(), "Exiting demo.");
//     rclcpp::shutdown();
// }