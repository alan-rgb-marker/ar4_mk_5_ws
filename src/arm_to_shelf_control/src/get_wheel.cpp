#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vision_interfaces/srv/armcoodinate.hpp>

using namespace std::chrono_literals;

class PickAndPlaceNode : public rclcpp::Node
{
public:
    PickAndPlaceNode() : Node("pick_and_place_node") {}

    void run()
    {
        auto arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm");
        auto gripper = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "gripper");

        // ── Step 1：移動到指定點 ──────────────────────────────────
        RCLCPP_INFO(get_logger(), "Step 1: Moving to target pose...");
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = 0.000;
        target_pose.position.y = -0.215;
        target_pose.position.z = 0.287;
        target_pose.orientation.x = 1.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;
        target_pose.orientation.w = 0.0;

        arm->setPoseTarget(target_pose);
        if (!planAndExecute(arm, "move to point"))
            return;

        // ── Step 2：等待 ──────────────────────────────────────────
        RCLCPP_INFO(get_logger(), "Step 2: Waiting 2 seconds...");
        rclcpp::sleep_for(1s);

        // ── Step 4：夾爪打開 ──────────────────────────────────────
        RCLCPP_INFO(get_logger(), "Step 4: Opening gripper...");
        gripper->setNamedTarget("open");
        if (!planAndExecute(gripper, "open"))
            return;

        // ── Step 3：呼叫 Service 取得夾取座標 ────────────────────
        RCLCPP_INFO(get_logger(), "Step 3: Requesting grasp pose from service...");
        auto client = create_client<vision_interfaces::srv::Armcoodinate>("wheel_pose");

        auto get_wheel_request = std::make_shared<vision_interfaces::srv::Armcoodinate::Request>();

        get_wheel_request->result = "get_wheel_pose";
        while (!client->wait_for_service())
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto future_reponse = client->async_send_request(get_wheel_request);

        geometry_msgs::msg::Pose grasp_pose = future_reponse.get()->arm_cood; // ← 欄位名你自己改

        RCLCPP_INFO(get_logger(), "Grasp pose: (%.3f, %.3f, %.3f)",
                    grasp_pose.position.x,
                    grasp_pose.position.y,
                    grasp_pose.position.z);
        RCLCPP_INFO(get_logger(), "Grasp orientation: (%.3f, %.3f, %.3f, %.3f)",
                    grasp_pose.orientation.x,
                    grasp_pose.orientation.y,
                    grasp_pose.orientation.z,
                    grasp_pose.orientation.w);
        // ── Step 5：移動到夾取座標 ────────────────────────────────
        RCLCPP_INFO(get_logger(), "Step 5: Moving to grasp pose...");
        rclcpp::sleep_for(3s);
        arm->setPoseTarget(grasp_pose);
        if (!planAndExecute(arm, "move to grasp pose"))
            return;

        // ── Step 6：夾爪關閉 ──────────────────────────────────────
        RCLCPP_INFO(get_logger(), "Step 6: Closing gripper...");
        gripper->setNamedTarget("close");
        if (!planAndExecute(gripper, "close"))
            return;

        grasp_pose.position.z += 0.05;
        arm->setPoseTarget(grasp_pose);
        if (!planAndExecute(arm, "move to grasp pose"))
            return;

        // ── Step 7：回到原點 ──────────────────────────────────────
        RCLCPP_INFO(get_logger(), "Step 7: Returning to ready...");
        arm->setNamedTarget("ready");
        if (!planAndExecute(arm, "return to ready"))
            return;

        RCLCPP_INFO(this->get_logger(), "Step 8: request shelf");
        auto shelf_client = create_client<std_srvs::srv::Trigger>("run_service");

        auto shelf_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        while (!shelf_client->wait_for_service())
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }
        auto shelf_reponse = shelf_client->async_send_request(shelf_request);

        RCLCPP_INFO(get_logger(), "Done!");
    }

private:
    bool planAndExecute(
        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group,
        const std::string &label)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (group->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "[FAIL] Planning: %s", label.c_str());
            return false;
        }
        if (group->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "[FAIL] Execution: %s", label.c_str());
            return false;
        }
        RCLCPP_INFO(get_logger(), "[OK] %s", label.c_str());
        return true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickAndPlaceNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread run_thread([&node]()
                           { node->run(); });

    executor.spin();
    run_thread.join();

    rclcpp::shutdown();
    return 0;
}