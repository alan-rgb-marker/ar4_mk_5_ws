from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_param_builder import ParameterBuilder   # ← 加上這行
from launch.actions import ExecuteProcess

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ar4").to_moveit_configs()

    # 正確載入 servo 參數（重要！）
    servo_yaml = ParameterBuilder("ar4_arm_servo_control") \
        .yaml("config/servo.yaml") \
        .to_dict()
    servo_params = {f"moveit_servo.{k}": v for k, v in servo_yaml.items()}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,                                   
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    init_servo_mode = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "/servo_node/switch_command_type",
            "moveit_msgs/srv/ServoCommandType",
            "{command_type: 1}"
        ],
        output="screen"
    )

    return LaunchDescription([
        servo_node, 
        init_servo_mode
    ])