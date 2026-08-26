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

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

#------------------CPP---------------------------
include_directories(include)

add_executable(test_planner src/test_planner.cpp)
ament_target_dependencies(test_planner ${dependencies})

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  test_planner
  DESTINATION lib/${PROJECT_NAME}
)
#-----------------------------------------------

#------PYTHON--------------------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/planner_benchmark.py
  DESTINATION lib/${PROJECT_NAME}
)
#---------------------------------------------

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
