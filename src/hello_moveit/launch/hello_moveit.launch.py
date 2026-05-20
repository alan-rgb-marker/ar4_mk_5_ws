from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 這裡會自動處理 ar4_description 的 URDF, ar4_moveit_config 的 SRDF 和 Kinematics
    # 無法手動拼湊路徑，這樣最穩定
    moveit_config = (
        MoveItConfigsBuilder("ar4", package_name="ar4_moveit_config")
        .to_moveit_configs()
    )

    # 建立 hello_moveit 節點
    moveit_demo_node = Node(
        package="hello_moveit",
        executable="hello_moveit",
        output="screen",
        parameters=[
            moveit_config.to_dict(), # 這裡包含所有運動學和模型參數
            {"use_sim_time": True}   # 給 Gazebo 仿真用
        ],
    )

    return LaunchDescription([
        moveit_demo_node
    ])