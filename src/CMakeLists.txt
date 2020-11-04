#
# Write here your robot name in order to build it..
#

set(all_src_targets)

set(ROBOT_NAMES solo8 solo8ti solo12 teststand)

set(robot_properties_package_names
    robot_properties_solo robot_properties_solo robot_properties_solo
    robot_properties_teststand)

set(dynamic_graph_manager_yaml_file
    dynamic_graph_manager/dgm_parameters_solo8.yaml
    dynamic_graph_manager/dgm_parameters_solo8ti.yaml
    dynamic_graph_manager/dgm_parameters_solo12.yaml
    dynamic_graph_manager/dgm_parameters.yaml)

#
# Declare a simple macro to build the library.
#

macro(create_dgm_library robot_name)
  set(lib_name dgm_${robot_name})
  # create library
  add_library(${lib_name} SHARED ${lib_name}.cpp)
  # Add the include dependencies.
  target_include_directories(
    ${lib_name} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(${lib_name} ament_index_cpp::ament_index_cpp)
  target_link_libraries(${lib_name} blmc_drivers::blmc_drivers)
  target_link_libraries(${lib_name} blmc_robots::${robot_name})
  target_link_libraries(${lib_name}
                        dynamic_graph_manager::dynamic_graph_manager)
  target_link_libraries(${lib_name} yaml_utils::yaml_utils)

  list(APPEND all_src_targets ${lib_name})
endmacro(create_dgm_library lib_name)

#
# Declare a simple macro to build the demos.
#

macro(create_dgm_executable robot_name package_name resource_path)

  set(executable_name dg_main_${robot_name})

  # Robot executables.
  add_executable(${executable_name} robot_executable/${robot_name}.cpp)
  # Add the include dependencies.
  target_include_directories(
    ${executable_name} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                              $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(${executable_name} dgm_${robot_name})
  # Some compile definitions
  target_compile_definitions(
    ${executable_name}
    PUBLIC target_compile_definitions
           (demo_${demo_name} PUBLIC
            ROBOT_PROPERTIES_PACKAGE_NAME="${package_name}"
            ROBOT_PROPERTIES_YAML_PATH="${resource_path}"))

  list(APPEND all_src_targets ${lib_name})
endmacro(create_dgm_executable robot_name)

#
# build the DGMs and the executable
#

list(LENGTH ROBOT_NAMES nb_names_tmp)
math(EXPR nb_names_tmp "${nb_names_tmp} - 1")

foreach(i RANGE ${nb_names_tmp})
  list(GET ROBOT_NAMES ${i} robot_name)
  list(GET ROBOT_PROPERTIES_PACKAGE_NAMES ${i} package_name)

  create_dgm_library(${robot_name})
  create_dgm_executable(${robot_name} ${robot_config_file})
endforeach()

#
# Install and Export the libraries.
#
install(
  TARGETS ${all_src_targets}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)
