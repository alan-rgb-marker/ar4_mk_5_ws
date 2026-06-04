import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ar4")
        .to_moveit_configs()
    )

    # Get parameters for the Servo node and namespace them correctly.
    servo_yaml = (
        ParameterBuilder("arm_to_shelf_control")
        .yaml("config/servo.yaml")
        .to_dict()
    )
    servo_params = {f"moveit_servo.{k}": v for k, v in servo_yaml.items()}

    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "arm"}

    
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = launch_ros.actions.Node(
        package="arm_to_shelf_control",
        executable="arm_to_shelf_control_node",
        parameters=[
            servo_params,
            # acceleration_filter_update_period,
            # planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            servo_node,
        ]
    )