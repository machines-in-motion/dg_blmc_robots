######################
# set up the project #
######################
cmake_minimum_required(VERSION 2.8.3)

project(dg_blmc_robots)

# required to use std::shared_ptr in code
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

############################
# Search for dependencies. #
############################
set(CATKIN_PKGS ${CATKIN_PKGS}
  robot_properties_quadruped
  robot_properties_teststand
#  robot_properties_stuggihop
  blmc_drivers
  blmc_robots
  #yaml_cpp
  dynamic_graph_manager
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})
search_for_python()
search_for_boost()
search_for_eigen()

# Trigger dependency to dynamic-graph.
add_required_dependency("yaml-cpp >= 0.5")
add_required_dependency("dynamic-graph >= 3.0.0")
add_required_dependency("dynamic-graph-python >= 3.0.0")
build_prefix_for_pkg("yaml-cpp" yaml_cpp_name)
build_prefix_for_pkg("dynamic-graph" dg_name)
build_prefix_for_pkg("dynamic-graph-python" dgp_name)
set(catkin_INCLUDE_DIRS
  ${${yaml_cpp_name}_INCLUDE_DIRS}
  ${${dg_name}_INCLUDE_DIRS}
  ${${dgp_name}_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(
  ${${yaml_cpp_name}_LIBRARY_DIRS}
  ${${dg_name}_LIBRARY_DIRS}
  ${${dgp_name}_LIBRARY_DIRS}
)
set(catkin_LIBRARIES
  ${${yaml_cpp_name}_LIBRARIES}
  ${${dg_name}_LIBRARIES}
  ${${dgp_name}_LIBRARIES}
  ${catkin_LIBRARIES}
)

define_os()

#########################################
# Declare the service files to be built #
#########################################
add_service_files(FILES
  TeststandCalibration.srv
)

###############################
# Generate the catkin package #
###############################
catkin_python_setup()

#####################################################################
# Actually generate the language-specific message and service files #
#####################################################################
generate_messages(DEPENDENCIES std_msgs)

######################################################
# define the include directory of all ${CATKIN_PKGS} #
######################################################
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_PATH}
    ${Boost_INCLUDE_DIR}
    ${Eigen_INCLUDE_DIRS}
)

########################################################
# manage the creation of the libraries and executables #
########################################################
add_subdirectory(src)

##########################################
# export the package as a catkin package #
##########################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${EXPORTED_LIBRAIRIES}
)

#########################
# manage the unit tests #
#########################
add_subdirectory(tests)

####################
# manage the demos #
####################
add_subdirectory(demos)

##########################
# building documentation #
##########################
build_doxygen_documentation()