from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    use_sim_time_cfg = LaunchConfiguration('use_sim_time')
    
    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation time'
        )

    get_wheel_from_shelf_node = Node(
        package='arm_to_shelf_control',
        executable='get_wheel_from_shelf_node',
        name='get_wheel_from_shelf_node',
        parameters=[{
                'use_sim_time': use_sim_time_cfg
            }]
    )

    detect_wheel_from_shelf = Node(
        package='vision_yolo_depth',
        executable='detect_wheel_from_shelf',
        name='detect_wheel_from_shelf',
        parameters=[{
                'use_sim_time': use_sim_time_cfg
            }],
        output='screen'
    )
    
    moveit_servo_service = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("arm_to_shelf_control"),
                    "launch",
                    "ar4_servo.launch.py",
                ]
            ),
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        detect_wheel_from_shelf,
        moveit_servo_service,
        get_wheel_from_shelf_node,
    ])