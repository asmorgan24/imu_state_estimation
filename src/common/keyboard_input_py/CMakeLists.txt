cmake_minimum_required(VERSION 2.8.3)
project(keyboard_input_py)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS 
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
