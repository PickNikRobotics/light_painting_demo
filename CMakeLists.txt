cmake_minimum_required(VERSION 3.16.3)
project(light_painting_demo)

# Check to see if this is set already
if(NOT "${CMAKE_CXX_STANDARD}")
    set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

# Support new yaml-cpp API.
find_package(PkgConfig)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)
include_directories(SYSTEM ${YAMLCPP_INCLUDE_DIRS})


set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
)

add_library(parse_image_waypoints SHARED src/parse_image_waypoints.cpp)
target_link_libraries(parse_image_waypoints ${YAMLCPP_LIBRARIES})
target_include_directories(parse_image_waypoints PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIR}
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(parse_image_waypoints ${THIS_PACKAGE_INCLUDE_DEPENDS} ${Eigen_LIBRARIES})

add_executable(waypoint_parsing_demo src/waypoint_parsing_demo.cpp)
target_link_libraries(waypoint_parsing_demo parse_image_waypoints)
target_include_directories(waypoint_parsing_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(waypoint_parsing_demo ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(run_demo src/run_demo.cpp)
target_link_libraries(run_demo parse_image_waypoints)
target_include_directories(waypoint_parsing_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(run_demo ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Modern cmake advises to add compile commands on a target basis.
# -Wall and -Wextra are standard but additional commands should be added at the command line.
# target_compile_options(my_library PRIVATE -Wall -Wextra)
# target_compile_options(my_node PRIVATE -Wall -Wextra)

# Export targets
#ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export dependencies to downstream packages
#ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS parse_image_waypoints
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install Binaries
install(TARGETS run_demo waypoint_parsing_demo
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch, config directories
install(
  DIRECTORY
    launch
    config
    resources
  DESTINATION share/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# Must call once per package
ament_package()
