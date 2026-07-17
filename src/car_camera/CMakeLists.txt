cmake_minimum_required(VERSION 3.8)
project(car_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# ---- car_camera 节点 ----
add_executable(car_camera_node src/car_camera_node.cpp)
ament_target_dependencies(car_camera_node
  rclcpp sensor_msgs cv_bridge)
target_link_libraries(car_camera_node ${OpenCV_LIBS})

install(TARGETS car_camera_node
  DESTINATION lib/${PROJECT_NAME})

# ---- launch / config ----
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# ---- 测试 ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 关闭 copyright/cpplint(暂无版权头);保留其余
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
