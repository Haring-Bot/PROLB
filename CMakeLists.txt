cmake_minimum_required(VERSION 3.0.2)
project(prolb_haring)

## Add your libraries which you need for this package here
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

## Use add_message_files() if you are working with your own message
# See this tutorial: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
#add_message_files(
#  FILES
#  mymessage.msg
#)

## Use add_service_files() if you are working with your own service
# See this tutorial: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
# add_service_files(
#   FILES
#   Service1.srv
# )

## Use add_action_files() if you are working with your own action
# See this tutorial: http://wiki.ros.org/actionlib/Tutorials
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Adapt generate_messages() for the ROS message inside your own message
# See this tutorial: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

## Use generate_dynamic_reconfigure_options() if you are working with your own dynamic reconfigure
# See this tutorial: http://wiki.ros.org/dynamic_reconfigure
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

# Declare the library and header packages needed
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_ros_package
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Add the include directories
include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(${PROJECT_NAME}
#   include/${PROJECT_NAME}/robot_class.cpp
#)

## For each node use add_executable() and link the libraries
add_executable(my_class_node src/master.cpp include/${PROJECT_NAME}/toolbox.cpp)
target_link_libraries(my_class_node ${catkin_LIBRARIES})
