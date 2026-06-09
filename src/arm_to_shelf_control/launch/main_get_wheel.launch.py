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
            default_value='True',
            description='Use simulation time'
        )

    main_get_wheel_node = Node(
        package='arm_to_shelf_control',
        executable='get_wheel_node',
        name='get_wheel_node',
        parameters=[{
                'use_sim_time': use_sim_time_cfg
            }]
    )

    wheel_pose_detect = Node(
        package='vision_yolo_depth',
        executable='depth_camera',
        name='depth_camera',
        parameters=[{
                'use_sim_time': use_sim_time_cfg
            }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        wheel_pose_detect,
        main_get_wheel_node,
    ])