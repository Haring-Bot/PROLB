cmake_minimum_required(VERSION 3.8)
project(prolb_haring)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add executable
add_executable(KF src/KF.cpp)
ament_target_dependencies(KF 
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(KF Eigen3::Eigen)

# Add navnode executable
add_executable(navnode src/navnode.cpp)
ament_target_dependencies(navnode rclcpp rclcpp_action nav2_msgs)

# Install executables
install(TARGETS
  KF
  navnode
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install map files
install(DIRECTORY
  map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
