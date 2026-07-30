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

add_executable(pure_pursuit_controller src/pure_pursuit_controller.cpp)
ament_target_dependencies(pure_pursuit_controller ${dependencies})

add_executable(a_star_smooth_planner src/a_star_smooth_planner.cpp)
ament_target_dependencies(a_star_smooth_planner ${dependencies})

add_executable(string_bezier_smoother src/string_bezier_smoother.cpp)
ament_target_dependencies(string_bezier_smoother ${dependencies})

add_executable(theta_star_planner src/theta_star_planner.cpp)
ament_target_dependencies(theta_star_planner ${dependencies})

add_executable(lazy_theta_star_planner src/lazy_theta_star_planner.cpp)
ament_target_dependencies(lazy_theta_star_planner ${dependencies})

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  pure_pursuit_controller
  a_star_smooth_planner
  string_bezier_smoother
  theta_star_planner
  lazy_theta_star_planner
  DESTINATION lib/${PROJECT_NAME}
)
#-----------------------------------------------


#------------------PYTHON-----------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/a_star_smooth_planner.py
  ${PROJECT_NAME}/theta_star_planner.py
  ${PROJECT_NAME}/lazy_theta_star_planner.py
  ${PROJECT_NAME}/drp_theta_star_planner.py
  ${PROJECT_NAME}/string_pull_bezier_smoother.py
  ${PROJECT_NAME}/pure_pursuit_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
#-----------------------------------------------

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
