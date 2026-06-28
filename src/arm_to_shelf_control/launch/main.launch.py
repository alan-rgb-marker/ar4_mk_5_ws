from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    use_sim_time_cfg = LaunchConfiguration('use_sim_time')
    
    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation time'
        )

    wheel_pose_detect = Node(
        package='vision_yolo_depth',
        executable='main_service',
        name='main_service',
        parameters=[{
                'use_sim_time': use_sim_time_cfg
            }]
    )
    
    main_get_wheel_node = Node(
        package='arm_to_shelf_control',
        executable='get_wheel_node',
        name='get_wheel_node',
        parameters=[{
                'use_sim_time': use_sim_time_cfg
            }]
    )
    
    main_arm_to_shelf_control_node = Node(
        package='arm_to_shelf_control',
        executable='arm_to_shelf_control_node',
        # name='arm_to_shelf_control_node',
        parameters=[{
                'use_sim_time': use_sim_time_cfg
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
            ),
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )
    
    call_start_service = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/start_run_service', 'std_srvs/srv/Trigger', '{}'],
        output='screen'
    )
    
    # ============================= 第一部份 =============================
    
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
    
    # ============================= 第二部份 =============================

    return LaunchDescription([
        use_sim_time_arg,
        wheel_pose_detect,
        main_get_wheel_node,
        main_arm_to_shelf_control_node,
        moveit_servo_service,
        call_start_service,
        # ========= 第一部份 ===========
        get_wheel_from_shelf_node,
        detect_wheel_from_shelf
    ])
    
#     use_sim_time_arg = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='False',
#         description='Use simulation (Gazebo) clock if true'
#     )
    
#     use_sim_time_val = LaunchConfiguration('use_sim_time')

#     move_group_include = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution(
#                 [
#                     FindPackageShare("ar4_moveit_config"),
#                     "launch",
#                     "move_group.launch.py",
#                 ]
#             )
#         )
#     )

#     get_wheel_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([
#                 FindPackageShare('arm_to_shelf_control'),
#                 'launch', 
#                 'main_get_wheel.launch.py'   # 修正：原程式碼多了 .launch.launch.py
#             ])
#         ),
        
#         launch_arguments={'use_sim_time': use_sim_time_val}.items()
#     )


#     arm_to_shelf_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([
#                 FindPackageShare('arm_to_shelf_control'),
#                 'launch', 
#                 'main_arm_to_shelf.launch.py' # 修正：原程式碼多了 .launch.launch.py
#             ])
#         ),
        
#         launch_arguments={'use_sim_time': use_sim_time_val}.items()
#     )
    
#     ar4_servo_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([
#                 FindPackageShare('arm_to_shelf_control'),
#                 'launch', 
#                 'ar4_servo.launch.py' # 修正：原程式碼多了 .launch.launch.py
#             ])
#         ),
#         launch_arguments={'use_sim_time': use_sim_time_val}.items()
#     )

#     return LaunchDescription([
#         use_sim_time_arg,
#         # move_group_include,
#         get_wheel_launch,
#         arm_to_shelf_launch,
#         ar4_servo_launch
#     ])