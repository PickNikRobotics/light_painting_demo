cmake_minimum_required(VERSION 3.16.3)
project(light_painting_demo)

# Check to see if this is set already
if(NOT "${CMAKE_CXX_STANDARD}")
    set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Place build_depends here
find_package(ament_cmake REQUIRED)
# find_package(<build_depend_pkg> REQUIRED)

# set(THIS_PACKAGE_INCLUDE_DEPENDS <build_depend_pkg>)

# Best practice: place only .c/.cpp files in add_library and add_executable
# Add a library
# add_library(my_library SHARED src/my_lib.cpp)

# Add an executable
# add_executable(my_node src/my_node.cpp)

# Place header files for your target here. Your target is a library or executable.
# target_include_directories(my_library PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )

# The recommended way in modern CMake is to only use targets, exporting and linking against them.
# The recommended way to link a target to a dependency is to use ament_target_dependencies.
# ament_target_dependencies(my_library ${THIS_PACKAGE_INCLUDE_DEPENDS})
# ament_target_dependencies(my_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Modern cmake advises to add compile commands on a target basis.
# -Wall and -Wextra are standard but additional commands should be added at the command line.
# target_compile_options(my_library PRIVATE -Wall -Wextra)
# target_compile_options(my_node PRIVATE -Wall -Wextra)

# Export targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export dependencies to downstream packages
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
# install(
#   TARGETS my_library
#   EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

# Install Binaries
# install(
#   TARGETS my_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch, config directories
install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# Must call once per package
ament_package()
