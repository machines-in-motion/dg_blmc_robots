################################################################
# create the dynamic_graph_manager for the 8 motors test bench #
################################################################

set(SOURCES
  dg_test_bench_8_motors/dg_test_bench_8_motors.cpp
)
add_library(dg_test_bench_8_motors SHARED ${SOURCES})
target_link_libraries(dg_test_bench_8_motors
  ${catkin_LIBRARIES}
)

if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  target_link_libraries(dg_test_bench_8_motors
    rt
    pthread
  )
endif(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
