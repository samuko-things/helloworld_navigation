cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp)
find_package(rclpy)
find_package(nav_msgs)
find_package(geometry_msgs)
find_package(tf2 REQUIRED)
find_package(tf2_ros)
find_package(tf2_geometry_msgs REQUIRED)

#---------------ADD FILES-----------------------
install(DIRECTORY
    config
    launch
    maps
    DESTINATION share/${PROJECT_NAME}/
)
#----------------------------------------------


ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
