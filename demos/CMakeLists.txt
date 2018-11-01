#####################
# create some demos #
#####################

include_directories(
  ${PROJECT_SOURCE_DIR}/demos
)

### demo off the 8 blmc motor test bench ####
add_executable(demo_dg_test_bench_8_motors
  dg_test_bench_8_motors/demo_dg_test_bench_8_motors.cpp
)

target_link_libraries(demo_dg_test_bench_8_motors
  dg_test_bench_8_motors
  ${catkin_LIBRARIES}
)
