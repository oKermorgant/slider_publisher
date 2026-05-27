cmake_minimum_required(VERSION 3.15)
project(slider_publisher)

find_package(ament_cmake REQUIRED)

install(DIRECTORY examples
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS src/slider_publisher
        DESTINATION lib/${PROJECT_NAME})

ament_package()
