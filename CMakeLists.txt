cmake_minimum_required(VERSION 2.8.3)
project(dvrk_spacenav)

include_directories(include ${catkin_INCLUDE_DIRS})

find_package(catkin REQUIRED 
          roslaunch
          rospy
          message_generation
          dvrk_python
          )

catkin_package(CATKIN_DEPENDS message_runtime rospy dvrk_python) 

roslaunch_add_file_check(launch)

catkin_install_python(PROGRAMS src/dvrk_spacenav.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
