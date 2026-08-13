cmake_minimum_required(VERSION 3.5)
project(lazy_theta_planner)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# # ====================================================================
# # PERFORMANCE OPTIMIZATION FLAGS
# # ====================================================================
# # Default to Release mode (-O3) if no build type is specified by colcon
# if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
#   set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
# endif()

# # Apply high-optimization compiler flags for Release builds
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
#   add_compile_options($<$<CONFIG:Release>:-O3>)
#   add_compile_options($<$<CONFIG:Release>:-march=native>)
#   add_compile_options($<$<CONFIG:Release>:-ffast-math>)
# endif()
# # ====================================================================

nav2_package()

include_directories(
  include
)

set(library_name lazy_theta_planner)

set(dependencies
  angles
  rclcpp
  rclcpp_components
  rclcpp_action
  std_msgs
  nav2_msgs
  nav2_util
  nav2_core
  nav_2d_utils
  nav_2d_msgs
  pluginlib
)

# Main plugin library
add_library(${library_name} SHARED
  src/lazy_theta_planner.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(
  TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()