from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # 1. 定義第一個要執行的 Launch 檔案
    arm_to_shelf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_to_shelf_control'),
                'launch', 
                'main_arm_to_shelf.launch.py' # 修正：原程式碼多了 .launch.launch.py
            ])
        )
    )

    # 2. 定義第二個要接續執行的 Launch 檔案
    main_get_wheel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_to_shelf_control'),
                'launch', 
                'main_get_wheel.launch.py'   # 修正：原程式碼多了 .launch.launch.py
            ])
        )
    )

    # 3. ⚠️ 關鍵修改：改成秒數等待 (例如等待 5.0 秒)
    # period 的單位是秒，你可以改成你想要的任何數字，比如 5.0, 10.0 等
    delay_second_launch = TimerAction(
        period=5.0, 
        actions=[
            LogInfo(msg="已等待 5 秒，現在開始啟動第二個 Launch..."),
            main_get_wheel_launch
        ]
    )

    # 4. 回傳 Launch 敘述
    # 這裡把第一個 launch 和 定時器一起塞進去
    return LaunchDescription([
        arm_to_shelf_launch,
        delay_second_launch
    ])