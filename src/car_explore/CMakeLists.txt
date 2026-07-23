cmake_minimum_required(VERSION 3.8)
project(car_explore)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

install(PROGRAMS
  scripts/roadmap_explore_mission.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
