cmake_minimum_required(VERSION 3.8)
project(dual_xarm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/single_arm_motions.py DESTINATION lib/${PROJECT_NAME})

ament_package()
