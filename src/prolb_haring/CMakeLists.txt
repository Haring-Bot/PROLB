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
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_package(ament_index_cpp REQUIRED)

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
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

add_executable(EKF src/EKF.cpp)
ament_target_dependencies(EKF 
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)

add_executable(PF src/PF.cpp)
ament_target_dependencies(PF
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  Eigen3
  ament_index_cpp
)

target_link_libraries(KF Eigen3::Eigen ${YAML_CPP_LIBRARIES})
target_include_directories(KF PRIVATE ${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(EKF Eigen3::Eigen ${YAML_CPP_LIBRARIES})
target_include_directories(EKF PRIVATE ${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(PF
  yaml-cpp
)

# Add navnode executable
add_executable(navnode src/navnode.cpp)
ament_target_dependencies(navnode rclcpp rclcpp_action nav2_msgs)

# Install executables
install(TARGETS
  KF
  EKF
  PF
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

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
