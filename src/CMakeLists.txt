####################################################
# Write here your robot name in order to build it. #
####################################################

set(ROBOT_NAMES
    solo8
    solo8ti
    solo12
    solo_simple_simu
    teststand
    stuggihop
)

set(ROBOT_CONFIG_FILES
    "${robot_properties_solo_SOURCE_PREFIX}/config/dgm_parameters_solo8.yaml"
    "${robot_properties_solo_SOURCE_PREFIX}/config/dgm_parameters_solo8ti.yaml"
    "${robot_properties_solo_SOURCE_PREFIX}/config/dgm_parameters_solo12.yaml"
    "${CMAKE_CURRENT_SOURCE_DIR}/config/solo_simple_simu.yaml"
    "${robot_properties_teststand_SOURCE_PREFIX}/config/dgm_parameters.yaml"
    "${robot_properties_stuggihop_SOURCE_PREFIX}/config/stuggihop.yaml"
)

###############################################
# Declare a simple macro to build the library #
###############################################

macro(create_dgm_library robot_name)
    set(lib_name dgm_${robot_name})

    add_library(${lib_name} SHARED ${lib_name}.cpp)

    target_link_libraries(${lib_name} ${catkin_LIBRARIES})

    if(TARGET ${robot_name})
      target_link_libraries(${lib_name} ${robot_name})
    endif()

    if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        target_link_libraries(${lib_name} rt pthread)
    endif()

    set(LIBRARIES_TO_BE_EXPORTED ${LIBRARIES_TO_BE_EXPORTED} ${lib_name})
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

    set(executable_name dg_main_${robot_name})
    if(NOT EXISTS ${robot_config_file})
        message(FATAL_ERROR "Error, cannot find the DGM config file: ${robot_config_file}")
    endif()
    target_compile_definitions(${executable_name} PUBLIC
                               YAML_PARAMS="${robot_config_file}")
endmacro(create_dgm_executable robot_name)

#####################################
# build the DGMs and the executable #
#####################################

list(LENGTH ROBOT_NAMES nb_names_tmp)
math(EXPR nb_names_tmp "${nb_names_tmp} - 1")

foreach(i RANGE ${nb_names_tmp})
  list(GET ROBOT_NAMES ${i} robot_name)
  list(GET ROBOT_CONFIG_FILES ${i} robot_config_file)

  create_dgm_library(${robot_name})
  create_dgm_executable(${robot_name} ${robot_config_file})
endforeach()


######################################################
# Export the libraries in the devel folder of catkin #
######################################################

set(EXPORTED_LIBRAIRIES ${EXPORTED_LIBRAIRIES} ${LIBRARIES_TO_BE_EXPORTED}
    PARENT_SCOPE)
