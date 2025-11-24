cmake_minimum_required(VERSION 3.8)
project(protobot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# INSTALL KE lib/${PROJECT_NAME} (ROS2 STANDARD UNTUK PYTHON)
install(
  PROGRAMS
    launch/pose_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# INSTALL LAUNCH FILES
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
  PATTERN "pose_manager_node.py" EXCLUDE
)

# INSTALL CONFIG
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  PROGRAMS
    launch/pose_manager_node.py
    launch/joystick_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

# INSTALL DESCRIPTION AND WORLDS
install(
  DIRECTORY description worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()