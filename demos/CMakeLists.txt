#####################
# create some demos #
#####################

include_directories(
  ${PROJECT_SOURCE_DIR}/demos
)

#############################################
# Declare a simple macro to build the demos #
#############################################

macro(create_dgm_demo robot_name)

### demo off the 8 blmc motor test bench ####
add_executable(demo_dg_${robot_name}
  demo_dg_${robot_name}.cpp
)

target_link_libraries(demo_dg_${robot_name}
  dgm_${robot_name}
  ${catkin_LIBRARIES}
)

# first look if a robot_properties_${robot_name} package contain a config file.
set(config_file
  "${robot_properties_${robot_name}_SOURCE_DIR}/config/${robot_name}.yaml"
)
# if this file do not exists then use one from the dg_blmc_robots package demos
if(NOT EXISTS ${config_file})
  set(config_file
    "${CMAKE_CURRENT_SOURCE_DIR}/config/${robot_name}.yaml"
  )
endif()

target_compile_definitions(demo_dg_${robot_name} PUBLIC
  YAML_PARAMS="${config_file}")

endmacro(create_dgm_demo robot_name)

###############
# build demos #
###############

create_dgm_demo(test_bench_8_motors)
create_dgm_demo(test_bench_8_motors_simu)
create_dgm_demo(quadruped)
create_dgm_demo(teststand)
create_dgm_demo(single_motor)
