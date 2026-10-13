cmake_minimum_required(VERSION 3.8)
project(ztr_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models urdf config DESTINATION share/${PROJECT_NAME})

ament_package()
