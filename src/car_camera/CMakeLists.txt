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
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)

# Apriltag C 库:优先用 find_package,否则回退 pkg-config
find_package(apriltag QUIET)
if(NOT apriltag_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(APRILTAG REQUIRED apriltag)
endif()

# ---- car_camera 节点 ----
add_executable(car_camera_node src/car_camera_node.cpp)
ament_target_dependencies(car_camera_node
  rclcpp sensor_msgs cv_bridge camera_info_manager)
target_link_libraries(car_camera_node ${OpenCV_LIBS})

# ---- apriltag 检测节点 ----
add_executable(apriltag_detector_node src/apriltag_detector_node.cpp)
ament_target_dependencies(apriltag_detector_node
  rclcpp sensor_msgs geometry_msgs tf2_ros cv_bridge)
target_link_libraries(apriltag_detector_node ${OpenCV_LIBS})
if(apriltag_FOUND)
  target_link_libraries(apriltag_detector_node apriltag::apriltag)
else()
  target_include_directories(apriltag_detector_node PRIVATE ${APRILTAG_INCLUDE_DIRS})
  target_link_libraries(apriltag_detector_node ${APRILTAG_LIBRARIES})
endif()

install(TARGETS car_camera_node apriltag_detector_node
  DESTINATION lib/${PROJECT_NAME})

# ---- 脚本 ----
install(PROGRAMS scripts/tag_follower.py
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
