#####################
# create some demos #
#####################

include_directories(
  ${PROJECT_SOURCE_DIR}/demos
)

### demo off the 8 blmc motor test bench ####
add_executable(demo_dg_test_bench_8_motors
  test_bench_8_motors/demo_dg_test_bench_8_motors.cpp
)

message(STATUS ${catkin_LIBRARIES})

target_link_libraries(demo_dg_test_bench_8_motors
  dg_test_bench_8_motors
  ${catkin_LIBRARIES}
)

target_compile_definitions(demo_dg_test_bench_8_motors PUBLIC
  YAML_PARAMS="${CMAKE_CURRENT_SOURCE_DIR}/test_bench_8_motors/test_bench_8_motors.yaml")
