cmake_minimum_required(VERSION 3.0.2)
project(slider_publisher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui_py
)

catkin_package()

catkin_install_python(PROGRAMS slider_publisher examples/dummy_gazebo_server.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY examples
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
