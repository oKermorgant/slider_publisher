cmake_minimum_required(VERSION 3.5)
project(slider_publisher)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
        examples
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
        src/slider_publisher.py
        src/dummy_gazebo_server.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
