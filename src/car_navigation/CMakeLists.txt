cmake_minimum_required(VERSION 3.8)
project(car_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps behavior_trees
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

install(PROGRAMS
  scripts/clearance_map.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_clearance_map
    test/test_clearance_map.py)
endif()

ament_package()
