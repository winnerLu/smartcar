cmake_minimum_required(VERSION 3.8)
project(car_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ---- car_base 节点 ----
add_executable(car_base_node src/car_base_node.cpp)
target_include_directories(car_base_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(car_base_node
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_ros)

install(TARGETS car_base_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

# ---- Python 脚本(验证工具 + 键盘遥控)----
install(PROGRAMS
  scripts/serial_check.py
  scripts/keyboard_teleop.py
  scripts/odom_monitor.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES
  scripts/protocol.py
  DESTINATION lib/${PROJECT_NAME})

# ---- 测试 ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 关闭 copyright/cpplint(暂无版权头);保留其余
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_include_directories(test_protocol PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()
