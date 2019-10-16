####################################################
# Write here your robot name in order to build it. #
####################################################

set(ROBOT_NAMES
    solo
    solo_simple_simu
    teststand
    stuggihop
)

###############################################
# Declare a simple macro to build the library #
###############################################

macro(create_dgm_library robot_name)
    cmake_policy(SET CMP0037 "OLD")
    cmake_policy(PUSH)

    set(lib_name dgm_${robot_name})

    add_library(${lib_name} SHARED ${lib_name}.cpp)

    target_link_libraries(${lib_name} ${catkin_LIBRARIES})

    if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(${lib_name} rt pthread)
    endif()

    set(LIBRARIES_TO_BE_EXPORTED ${LIBRARIES_TO_BE_EXPORTED} ${lib_name})

    cmake_policy(POP)
endmacro(create_dgm_library lib_name)

#############################################
# Declare a simple macro to build the demos #
#############################################

macro(create_dgm_executable robot_name)

    set(executable_name dg_main_${robot_name})

    ### demo off the 8 blmc motor test bench ####
    add_executable(${executable_name}
        robot_executable/${robot_name}.cpp
    )

    target_link_libraries(${executable_name}
        dgm_${robot_name}
        ${catkin_LIBRARIES}
    )

    # first look if a robot_properties_${robot_name} package contain a config file.
    set(config_file
        "${robot_properties_${robot_name}_SOURCE_DIR}/config/dgm_parameters.yaml"
    )
    # if this file do not exists then use one from the dg_blmc_robots package demos
    if(NOT EXISTS ${config_file})
        set(config_file
            "${CMAKE_CURRENT_SOURCE_DIR}/config/${robot_name}_dgm_parameters.yaml"
        )
    endif()

    target_compile_definitions(${executable_name} PUBLIC
    YAML_PARAMS="${config_file}")

endmacro(create_dgm_executable robot_name)

#####################################
# build the DGMs and the executable #
#####################################

FOREACH(ROBOT_NAME ${ROBOT_NAMES})
    create_dgm_library(${ROBOT_NAME})
    create_dgm_executable(${ROBOT_NAME})
ENDFOREACH(ROBOT_NAME ${ROBOT_NAMES})

######################################################
# Export the libraries in the devel folder of catkin #
######################################################

set(EXPORTED_LIBRAIRIES ${EXPORTED_LIBRAIRIES} ${LIBRARIES_TO_BE_EXPORTED}
    PARENT_SCOPE)
