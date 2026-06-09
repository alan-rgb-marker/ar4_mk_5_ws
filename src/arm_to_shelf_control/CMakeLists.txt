cmake_minimum_required(VERSION 3.8)
project(arm_to_shelf_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(vision_interfaces REQUIRED)
find_package(moveit_servo REQUIRED)

add_executable(arm_to_shelf_control_node src/arm_to_shelf_control.cpp)
target_include_directories(arm_to_shelf_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(arm_to_shelf_control_node
  rclcpp
  vision_interfaces
  moveit_servo
  moveit_ros_planning_interface
  moveit_servo
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(demo_shelf_moveit_node src/demo_shelf_moveit.cpp)
target_include_directories(demo_shelf_moveit_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(demo_shelf_moveit_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

add_executable(moveit_servo_service_node src/moveit_servo_service.cpp)
target_include_directories(moveit_servo_service_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(moveit_servo_service_node
  rclcpp
  moveit_servo
  geometry_msgs
)

add_executable(get_wheel_node src/get_wheel.cpp)
target_include_directories(get_wheel_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(get_wheel_node
  rclcpp
  moveit_servo
  geometry_msgs
  vision_interfaces
  std_srvs
)

install(TARGETS arm_to_shelf_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS demo_shelf_moveit_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS moveit_servo_service_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS get_wheel_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
