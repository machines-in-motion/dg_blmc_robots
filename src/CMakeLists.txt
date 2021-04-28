#
# Write here your robot name in order to build it..
#

set(all_src_targets)

set(robot_names solo8 solo8ti solo12)

set(dynamic_graph_manager_yaml_files
    ${PythonModules_robot_properties_solo_PATH}/robot_properties_solo/robot_properties_solo/dynamic_graph_manager/dgm_parameters_solo8.yaml
    ${PythonModules_robot_properties_solo_PATH}/robot_properties_solo/robot_properties_solo/dynamic_graph_manager/dgm_parameters_solo8ti.yaml
    ${PythonModules_robot_properties_solo_PATH}/robot_properties_solo/robot_properties_solo/dynamic_graph_manager/dgm_parameters_solo12.yaml
)

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
  ament_target_dependencies(${lib_name} mim_msgs)
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

macro(create_dgm_executable robot_name yaml_path)

  set(executable_name dg_main_${robot_name})

  # Robot executables.
  add_executable(${executable_name} robot_executable/${robot_name}.cpp)
  # Add the include dependencies.
  target_include_directories(
    ${executable_name}
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
           ${dynamic_graph_manager_INCLUDE_DIRS} $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(${executable_name} dgm_${robot_name})
  # Some compile definitions
  if(EXISTS ${yaml_path})
    target_compile_definitions(${executable_name}
                               PUBLIC ROBOT_PROPERTIES_YAML_PATH="${yaml_path}")
  else()
    message(FATAL_ERROR "The yaml path ${yaml_path} is invalid!")
  endif()
  list(APPEND all_src_targets ${executable_name})
endmacro(create_dgm_executable robot_name)

#
# build the DGMs and the executable
#

list(LENGTH robot_names nb_names_tmp)
math(EXPR nb_names_tmp "${nb_names_tmp} - 1")

foreach(i RANGE ${nb_names_tmp})
  list(GET robot_names ${i} robot_name)
  list(GET robot_properties_package_names ${i} robot_properties_package_name)
  list(GET dynamic_graph_manager_yaml_files ${i}
       dynamic_graph_manager_yaml_file)

  create_dgm_library(${robot_name})
  create_dgm_executable(${robot_name} ${dynamic_graph_manager_yaml_file})
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
