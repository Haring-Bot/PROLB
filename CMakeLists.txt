cmake_minimum_required(VERSION 3.0)
project(PROLB_Haring)

# Find OpenCV package
find_package(catkin REQUIRED
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    tf
    turtlebot3_gazebo
    turtlebot3_description
    turtlebot3_teleop
    rviz   
)

# Add executable target
add_executable(main src/main.cpp)

# Link OpenCV libraries
#target_link_libraries(main ${OpenCV_LIBS})
