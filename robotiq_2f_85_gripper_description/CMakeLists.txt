cmake_minimum_required(VERSION 2.8.3)
project(robotiq_2f_85_gripper_description)
find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY meshes urdf launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
