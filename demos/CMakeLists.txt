#####################
# create some demos #
#####################

include_directories(
  ${PROJECT_SOURCE_DIR}/demos
)

#############################################
# Declare a simple macro to build the demos #
#############################################

macro(create_dgm_demo demo_name)

### demo off the 8 blmc motor test bench ####
add_executable(demo_dg_${demo_name}
  demo_dg_${demo_name}.cpp
)

target_link_libraries(demo_dg_${demo_name}
  dgm_${demo_name}
  ${catkin_LIBRARIES}
)

target_compile_definitions(demo_dg_${demo_name} PUBLIC
  YAML_PARAMS="${CMAKE_CURRENT_SOURCE_DIR}/config/${demo_name}.yaml")

endmacro(create_dgm_demo demo_name)

###############
# build demos #
###############

create_dgm_demo(test_bench_8_motors)
create_dgm_demo(test_bench_8_motors_simu)
create_dgm_demo(quadruped)
