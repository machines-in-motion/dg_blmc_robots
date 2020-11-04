#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# Set up the project.
#
cmake_minimum_required(VERSION 3.10.2)

project(blmc_robots)

# Using C++17.
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#
# Dependencies
#

# Build tools.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(mpi_cmake_modules REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare the ROS message and service files to be built. This is here because
# otherwise the Python include_dir is not found...
rosidl_generate_interfaces(${PROJECT_NAME}_msg_srv "srv/JointCalibration.srv")

# Depend on ament macros.
find_package(ament_index_cpp REQUIRED)
find_package(blmc_drivers REQUIRED)
find_package(blmc_robots REQUIRED)
find_package(dynamic_graph_manager REQUIRED)
find_package(yaml_utils REQUIRED)

find_package(pinocchio_bullet REQUIRED)
find_package(robot_properties_solo REQUIRED)
find_package(robot_properties_teststand REQUIRED)

# Export de dependencies.
ament_export_dependencies(
  ament_cmake
  ament_cmake_python
  mpi_cmake_modules
  rosidl_default_generators
  ament_index_cpp
  blmc_drivers
  blmc_robots
  dynamic_graph_manager
  yaml_utils
  pinocchio_bullet
  robot_properties_solo
  robot_properties_teststand)

# Prepare the final export.
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#
# Manage the creation of the libraries and executables.
#
add_subdirectory(src)

#
# Manage the unit tests.
#
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

#
# Manage the demos.
#
add_subdirectory(demos)

#
# Install the package
#
install(DIRECTORY include/ DESTINATION include)

#
# Building documentation.
#
add_documentation()

#
# Export as an ament_package
#
ament_package()
