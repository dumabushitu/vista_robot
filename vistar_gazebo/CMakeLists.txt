cmake_minimum_required(VERSION 2.8.3)

project(vistar_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch controller models urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
