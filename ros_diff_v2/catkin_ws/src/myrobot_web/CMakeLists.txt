cmake_minimum_required(VERSION 3.0.2)
project(myrobot_web)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mobile_robot_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
