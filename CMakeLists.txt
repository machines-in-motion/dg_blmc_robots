#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# Set up the project.
#
cmake_minimum_required(VERSION 3.10.2)

project(dg_blmc_robots)

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
find_package(mim_msgs REQUIRED)
# Depend on ament macros.
find_package(blmc_drivers REQUIRED)
find_package(solo REQUIRED)
find_package(dynamic_graph_manager REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(PythonModules REQUIRED COMPONENTS robot_properties_solo)

# Export de dependencies.
ament_export_dependencies(
  ament_cmake
  ament_cmake_python
  mpi_cmake_modules
  mim_msgs
  ament_index_cpp
  blmc_drivers
  solo
  dynamic_graph_manager
  yaml_utils
  pinocchio_bullet
  robot_properties_solo)

# Prepare the final export.
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#
# Install the python package.
#
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})

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
# Install the package includes.
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
