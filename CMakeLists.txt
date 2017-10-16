cmake_minimum_required(VERSION 2.8.3)
project(slider_publisher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui_py
)

catkin_package()

catkin_install_python(PROGRAMS slider_publisher
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
