cmake_minimum_required(VERSION 3.8)
project(dual_xarm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/single_arm_motions.py
  scripts/charuco_detector.py
  scripts/calibrate_camera.py
  scripts/publish_calibration.py
  scripts/marker_hover_demo.py
  scripts/make_charuco_board.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
