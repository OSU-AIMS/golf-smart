cmake_minimum_required(VERSION 3.0.2)

project(golf_smart)

find_package(catkin REQUIRED
    COMPONENTS
        rospy
)

######################################
## Catkin Specific Configuration

catkin_package()

######################################
## Install

install(DIRECTORY config launch scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
