cmake_minimum_required(VERSION 3.8)
project(car_explore)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

install(PROGRAMS
  scripts/roadmap_explore_mission.py
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  scripts/roadmap_handoff.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_roadmap_handoff
    test/test_roadmap_handoff.py)
endif()

ament_package()
