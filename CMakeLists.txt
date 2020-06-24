######################
# set up the project #
######################
cmake_minimum_required(VERSION 2.8.3)

project(dg_blmc_robots)

# required to use std::shared_ptr in code and to link the python bindings
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed")
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

############################
# Search for dependencies. #
############################
set(CATKIN_PKGS ${CATKIN_PKGS}
  mpi_cmake_modules
  blmc_drivers
  blmc_robots
  yaml_cpp_catkin
  dynamic_graph_manager
  pinocchio_bullet
  robot_properties_solo
  robot_properties_teststand
  robot_properties_stuggihop
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

# Trigger dependency to dynamic-graph.
catkin_add_required_dependency("yaml-cpp")
catkin_add_required_dependency("dynamic-graph >= 3.0.0")
catkin_add_required_dependency("dynamic-graph-python >= 3.0.0")

search_for_boost()
search_for_eigen()
search_for_python()

#########################################
# Declare the service files to be built #
#########################################
add_service_files(FILES
  JointCalibration.srv
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
include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})

##########################################
# export the package as a catkin package #
##########################################
set(exported_libraries
  dgm_solo8
  dgm_solo8ti
  dgm_solo12
  dgm_solo_simple_simu
  dgm_teststand
  dgm_stuggihop
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${exported_libraries}
  CATKIN_DEPENDS ${CATKIN_PKGS}
)

########################################################
# manage the creation of the libraries and executables #
########################################################
add_subdirectory(src)

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