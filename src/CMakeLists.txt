###############################################
# Declare a simple macro to build the library #
###############################################

macro(create_dgm_library lib_name)

add_library(${lib_name} SHARED ${lib_name}.cpp)

target_link_libraries(${lib_name} ${catkin_LIBRARIES})

if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  target_link_libraries(${lib_name} rt pthread)
endif()

set(LIBRARIES_TO_BE_EXPORTED ${LIBRARIES_TO_BE_EXPORTED} ${lib_name})

endmacro(create_dgm_library lib_name)

##################
# build the DGMs #
##################

create_dgm_library(dgm_test_bench_8_motors)
create_dgm_library(dgm_test_bench_8_motors_simu)
create_dgm_library(dgm_quadruped)
create_dgm_library(dgm_teststand)
create_dgm_library(dgm_single_motor)
# create_dgm_library(dgm_stuggihop)

######################################################
# Export the libraries in the devel folder of catkin #
######################################################

set(EXPORTED_LIBRAIRIES ${EXPORTED_LIBRAIRIES} ${LIBRARIES_TO_BE_EXPORTED}
    PARENT_SCOPE)
