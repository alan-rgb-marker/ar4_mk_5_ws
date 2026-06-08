from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation time'
        )

    main_arm_to_shelf_control_node = Node(
        package='arm_to_shelf_control',
        executable='arm_to_shelf_control_node',
        name='arm_to_shelf_control_node',
        parameters=[{
                'use_sim_time': use_sim_time
            }]
    )

    shelf_pose_detect = Node(
        package='vision_yolo_depth',
        executable='shelf_pose_detect',
        name='shelf_pose_detect',
        parameters=[{
                'use_sim_time': use_sim_time
            }]
    )
    
    moveit_servo_service = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("arm_to_shelf_control"),
                    "launch",
                    "ar4_servo.launch.py",
                ]
            )
        )
    )

    return LaunchDescription([
        main_arm_to_shelf_control_node,
        shelf_pose_detect,
        moveit_servo_service
    ])