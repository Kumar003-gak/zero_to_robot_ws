cmake_minimum_required(VERSION 3.8)
project(ztr_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY launch worlds models DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/cmd_vel_guard.py
  scripts/mqtt_joystick_bridge.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
